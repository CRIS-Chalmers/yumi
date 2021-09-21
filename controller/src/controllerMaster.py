#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Int64
from controller.msg import Kinematics_msg, Trajectory_msg
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading
import rosservice
from abb_rapid_sm_addin_msgs.srv import SetSGCommand
from abb_robot_msgs.srv import TriggerWithResultCode

import message_filters
import time
import HQPSolver
import utils 
import Task
from controlTarget import ControlTarget
from parameters import Paramters

class YmuiContoller(object):
    def __init__(self):

        self.jointState = utils.JointState() # keeps track of joint postions 

        # class for generating target velocities
        self.controlTarget = ControlTarget(Paramters.dT)

        # Forward kinematics and transformers 
        # for critical updates
        self.yumiGrippPoseR = utils.FramePose()
        self.yumiGrippPoseL = utils.FramePose()
        self.yumiElbowPoseR = utils.FramePose()
        self.yumiElbowPoseL = utils.FramePose()

        self.tfListener = tf.TransformListener() # for non critical updates, no guarantee for synch
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
                
        #Class instance for gripper control
        self.gripperControl = utils.GripperControl() 
       
        stopEGM = rospy.ServiceProxy('/yumi/rws/sm_addin/stop_egm', TriggerWithResultCode)
        
        # HQP solver 
        self.HQP = HQPSolver.HQPSolver(stopEGM)

        # Task objects -------------
        # ctype (constraint type) 0 = equality, 1 = upper, -1 = lower
      
        jointPoistionBoundUpper = np.hstack([Paramters.jointPoistionBoundUpper, Paramters.jointPoistionBoundUpper]) # two arms
        jointPoistionBoundLower = np.hstack([Paramters.jointPoistionBoundLower, Paramters.jointPoistionBoundLower]) # two arms

        self.jointPositionBound = Task.JointPositionBoundsTaskV2(Dof=Paramters.Dof,\
                     boundsUpper=jointPoistionBoundUpper, boundsLower=jointPoistionBoundLower, timestep=Paramters.dT)

        # joint velocity limit 
        
        jointVelocityLimits = np.hstack([Paramters.jointVelocityBound, Paramters.jointVelocityBound]) # two arms

        self.jointVelocityBound = Task.JointVelocityBoundsTaskV2(Dof=Paramters.Dof,\
                     boundsUpper=jointVelocityLimits, boundsLower=-jointVelocityLimits)
        self.jointVelocityBound.compute() # constant
        # end effector collision avoidance
        self.endeEffectorCollision = Task.EndEffectorProximity(Dof=Paramters.Dof,\
                                    minDistance=Paramters.gripperMinDinstance, timestep=Paramters.dT)

        # Control objective
        
        self.indiviualControl = Task.IndividualControl(Dof=Paramters.Dof)
        self.absoluteControl = Task.AbsoluteControl(Dof=Paramters.Dof)
        self.relativeControl = Task.RelativeControl(Dof=Paramters.Dof)

        # elbow colision
       
        self.selfCollisionElbow = Task.ElbowProximityV2(Dof=Paramters.Dof,\
                                     minDistance=Paramters.minElbowDistance, timestep=Paramters.dT)
        # joint potential 
        self.jointPositionPotential = Task.JointPositionPotential(Dof=Paramters.Dof, defaultPose=Paramters.neutralPose,\
                                                                 timestep=Paramters.dT)
        
        # mutex
        self.lock = threading.Lock()
        
        # publish velocity comands
        self.pub = rospy.Publisher('/yumi/egm/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
        # publich current sengement on trajectory, used by the planner 
        self.pubSubTask = rospy.Publisher('/controller/sub_task', Int64, queue_size=1)


    def callback(self, data):

        # update joint position
        self.jointState.UpdatePose(pose=np.asarray(data.jointPosition))
        
        # distance from wrist to tip of grippers, ussually constant. Value for tf tree, can be set in in robot_setup_tf 
        (gripperLengthRight, _) = self.tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        (gripperLengthLeft, _) = self.tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))
        self.gripperLengthRight = np.asarray(gripperLengthRight)
        self.gripperLengthLeft = np.asarray(gripperLengthLeft)

        # calculated the combined geometric jacobian from base frame to tip of gripper for both arms
        jacobianCombined = utils.CalcJacobianCombined(data=data.jacobian[0], \
                                                    gripperLengthRight=gripperLengthRight,\
                                                    gripperLengthLeft=gripperLengthLeft,\
                                                    transformer=self.transformer,\
                                                    yumiGrippPoseR=self.yumiGrippPoseR,\
                                                    yumiGrippPoseL=self.yumiGrippPoseL)
        # Forward kinematics, Using KDL here instead of tf tree 
        self.yumiGrippPoseR.update(data.forwardKinematics[0], self.transformer, self.gripperLengthRight)
        self.yumiGrippPoseL.update(data.forwardKinematics[1], self.transformer, self.gripperLengthLeft)
        self.yumiElbowPoseR.update(data.forwardKinematics[2], self.transformer, np.zeros(3))
        self.yumiElbowPoseL.update(data.forwardKinematics[3], self.transformer, np.zeros(3))
        
        # stack of tasks, in decending hierarchy
        # ----------------------
        SoT = []
        # velocity bound
        SoT.append(self.jointVelocityBound)
        # position bound
        
        self.jointPositionBound.compute(jointState=self.jointState)
        SoT.append(self.jointPositionBound)

        # Elbow proximity limit task
        jacobianRightElbow = np.zeros((6,4))
        jacobianLeftElbow = np.zeros((6,4))
        dataNP = np.asarray(data.jacobian[1].data)
        jacobianRightElbow = dataNP[0::2].reshape((6,4))
        jacobianLeftElbow = dataNP[1::2].reshape((6,4))
      
        self.selfCollisionElbow.compute(jacobianRightElbow=jacobianRightElbow,\
                                        jacobianLeftElbow=jacobianLeftElbow,\
                                        yumiElbowPoseR=self.yumiElbowPoseR,\
                                        yumiElbowPoseL=self.yumiElbowPoseL)
        SoT.append(self.selfCollisionElbow)
        
        # Update the pose for controlTarget class  
        self.controlTarget.updatePose(yumiGrippPoseR=self.yumiGrippPoseR,\
                                                 yumiGrippPoseL=self.yumiGrippPoseL)
                                             
        self.lock.acquire() # mutex lock, each callback is on its own thread

        # calculates target velocities and positions
        self.controlTarget.updateTarget()

        if self.controlTarget.mode == 'individual':
            # Gripper colision avoidance 
            if Paramters.gripperColllsionActive:
                self.endeEffectorCollision.compute(jacobian=jacobianCombined,\
                                                    yumiPoseR=self.yumiGrippPoseR,\
                                                    yumiPoseL=self.yumiGrippPoseL)
                SoT.append(self.endeEffectorCollision)

            # Individual control objectiov 
            self.indiviualControl.compute(controlTarget=self.controlTarget, jacobian=jacobianCombined)
            SoT.append(self.indiviualControl)

        elif self.controlTarget.mode == 'coordinated':
            # Relative motion
            self.relativeControl.compute(controlTarget=self.controlTarget,\
                                        jacobian=jacobianCombined,\
                                        transformer=self.transformer)
            SoT.append(self.relativeControl) 
            # Absolute motion 
            self.absoluteControl.compute(controlTarget=self.controlTarget,\
                                        jacobian=jacobianCombined)
            SoT.append(self.absoluteControl) 

        else:
            print('Non valid control mode, stopping')
            self.jointState.jointVelocity = np.zeros(Paramters.Dof)
            self.lock.release()
            # publish velocity comands
            self.publishVelocity()
            return

        self.lock.release()

        # check so devation from the trajectory is not too big, stop if it is (turned of if gripperColllsionActive is active for indiviual mode)
        if not self.controlTarget.checkTrajectoryDevation():
            print('Deveation from trajectory too large, stopping')
            self.jointState.jointVelocity = np.zeros(Paramters.Dof)
            self.publishVelocity()
            return

        # Joint potential task, tries to keep the robot in a good configuration 
        self.jointPositionPotential.compute(jointState=self.jointState)
        SoT.append(self.jointPositionPotential)
        
        # solve HQP
        # ----------------------
        self.jointState.jointVelocity = self.HQP.solve(SoT=SoT)

        # gripper control
        # ----------------------
        # only called when a newsegment on the trajectory is entered 
        if self.controlTarget.checkNewTrajectorySegment():
            self.gripperControl.setGripperPosition(gripperRight=self.controlTarget.gripperRight,\
                                                gripperLeft=self.controlTarget.gripperLeft)
        
        # publish velocity comands
        self.publishVelocity()


    def publishVelocity(self):
        # sends the control comands to the driver 
        msg = Float64MultiArray()
        # Left Arm, Right Arm
        msg.data = np.hstack([self.jointState.GetJointVelocity()[7:14], self.jointState.GetJointVelocity()[0:7]]).tolist()
        self.pub.publish(msg)
        # sends information about which part of the trajectort is beeing executed 
        msgSubTask = Int64()
        msgSubTask.data = self.controlTarget.trajectory.index - 1
        self.pubSubTask.publish(msgSubTask)


    def callbackTrajectory(self, data):
        # Gets called when a new set of trajectory paramters is recived
        # The variable names in this funciton and the the trajectory class follows 
        # individual motion with left and right . This means when coordinate manipulation 
        # is usd, right is aboslute motion and left becomed relative motion. 
        
        # get current pose, used as first trajectory paramter 
        if data.mode == 'coordinated':
            positionRight = np.copy(self.controlTarget.absolutePosition)
            positionLeft  = np.copy(self.controlTarget.realativPosition) 
            orientationRight = np.copy(self.controlTarget.absoluteOrientation)
            orientationLeft = np.copy(self.controlTarget.rotationRelative)
        elif data.mode == 'individual':
            positionRight = np.copy(self.controlTarget.translationRightArm)
            positionLeft  = np.copy(self.controlTarget.translationLeftArm)
            orientationRight = np.copy(self.controlTarget.rotationRightArm)
            orientationLeft = np.copy(self.controlTarget.rotationLeftArm)
        else:
            print('Error, mode not matching combined or individual')
            return

        # current gripper position # 
        gripperLeft = self.controlTarget.gripperLeft
        gripperRight = self.controlTarget.gripperRight
        currentPoint =utils.TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight)
        trajectory = [currentPoint]

        # append trajectory points from msg
        for i in range(len(data.trajectory)):
            if data.mode == 'coordinated':
                positionRight = np.asarray(data.trajectory[i].positionAbsolute)
                positionLeft  = np.asarray(data.trajectory[i].positionRelative)
                orientationRight = np.asarray(data.trajectory[i].orientationAbsolute)
                orientationLeft = np.asarray(data.trajectory[i].orientationRelative)
            else:
                positionRight = np.asarray(data.trajectory[i].positionRight)
                positionLeft  = np.asarray(data.trajectory[i].positionLeft)
                orientationRight = np.asarray(data.trajectory[i].orientationRight)
                orientationLeft = np.asarray(data.trajectory[i].orientationLeft)

            gripperLeft = data.trajectory[i].gripperLeft
            gripperRight = data.trajectory[i].gripperRight
            pointTime = np.asarray(data.trajectory[i].pointTime)
            trajectroyPoint = utils.TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight,\
                                                    pointTime=pointTime)
            trajectory.append(trajectroyPoint)
        
        self.lock.acquire() # lock when updating trajectory       

        # use current velocity for smoother transitions, (only for translation)
        if self.controlTarget.mode == data.mode: # no change in control mode
            velLeftInit = np.copy(self.controlTarget.targetVelocities[6:9])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        elif self.controlTarget.mode == 'individual': # going from individual to coordinate motion
            # simple solution, not fully accurate trasition 
            velLeftInit = np.zeros(3)
            velRightInit = 0.5*(np.copy(self.controlTarget.targetVelocities[0:3]) +\
                                     np.copy(self.controlTarget.targetVelocities[6:9]))
        elif self.controlTarget.mode == 'coordinated': # going from coordinated to individual motion
            # simple solution, not fully accurate trasition 
            velLeftInit = np.copy(self.controlTarget.targetVelocities[0:3])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)
            
        # set mode
        self.controlTarget.mode = data.mode

        # update the trajectroy 
        self.controlTarget.trajectory.updateTrajectory(trajectory, velLeftInit, velRightInit)
        self.controlTarget.trajIndex = 0
        self.controlTarget.trajectorySegment = 0
        self.lock.release() 


def main():

    # starting ROS node and subscribers
    rospy.init_node('controller', anonymous=True) 

    ymuiContoller = YmuiContoller()
    rospy.sleep(0.05) # wait for every thing to initilize 
    rospy.Subscriber("/Jacobian_R_L", Kinematics_msg, ymuiContoller.callback, queue_size=3)
    rospy.Subscriber("/Trajectroy", Trajectory_msg, ymuiContoller.callbackTrajectory, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
