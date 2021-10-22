#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Int64
from controller.msg import Kinematics_msg, Trajectory_msg
import numpy as np
import tf
import threading
import rosservice
from abb_rapid_sm_addin_msgs.srv import SetSGCommand
from abb_robot_msgs.srv import TriggerWithResultCode

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import message_filters
import time
import HQPSolver
import utils 
import Task
from parameters import Paramters


class YumiController(object):
    def __init__(self):
        self.jointState = utils.JointState() # keeps track of joint postions 
        self.velocityCommand = utils.VelocityCommand()

        # Forward kinematics and transformers 
        # for critical updates
        self.yumiGrippPoseR = utils.FramePose()
        self.yumiGrippPoseL = utils.FramePose()
        self.yumiElbowPoseR = utils.FramePose()
        self.yumiElbowPoseL = utils.FramePose()

        # setup transformations 
        gripperRightLocal = utils.FramePose(position=np.array([0,0,0.143]), quaternion=np.array([0,0,0,1]))
        gripperLeftLocal =  utils.FramePose(position=np.array([0,0,0.143]), quaternion=np.array([0,0,0,1]))
        yumiToWorldLocal =  utils.FramePose(position=np.array([0.181,0,0]), quaternion=np.array([0,0,0.7071,-0.7071]))
        self.tfFrames = utils.TfBroadcastFrames(gripperRightLocal, gripperLeftLocal, yumiToWorldLocal)
        self.tfFrames.tfBroadcast()
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

        # mutex
        self.lock = threading.Lock()
        # stack of tasks
        self.data = ''
        #Class instance for gripper control
        self.gripperControl = utils.GripperControl() 
        self.setupTasks()

        # publish velocity comands
        self.pub = rospy.Publisher('/yumi/egm/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/Jacobian_R_L", Kinematics_msg, self.callback, queue_size=3, tcp_nodelay=True)


    def setupTasks(self):
        # HQP solver 
        stopEGM = rospy.ServiceProxy('/yumi/rws/sm_addin/stop_egm', TriggerWithResultCode)
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


    def callback(self, data):
        self.data = data
        # update joint position
        self.jointState.UpdatePose(pose=np.asarray(self.data.jointPosition))
        self.jointState.UpdateVelocity(velocity=np.asarray(self.data.jointPosition))

        # Forward kinematics, Using KDL here instead of tf tree 
        self.yumiGrippPoseR.update_(self.data.forwardKinematics[0], self.transformer, self.tfFrames.getGripperRight())
        self.yumiGrippPoseL.update_(self.data.forwardKinematics[1], self.transformer, self.tfFrames.getGripperLeft())
        
        self.yumiElbowPoseL.update_(self.data.forwardKinematics[3])
        self.yumiElbowPoseR.update_(self.data.forwardKinematics[2])
        
        #print('left', self.yumiElbowPoseL.getPosition(), 'right',self.yumiElbowPoseR.getPosition())
        #print('left ptr', self.yumiElbowPoseL.position.data, 'right ptr ',self.yumiElbowPoseR.position.data)
        #print('share memmory', np.may_share_memory(self.yumiElbowPoseL.position, self.yumiElbowPoseR.position))
        self.policy()


    def setAction(self, action):
        # joint control 
        if action['controlSpace'] == 'JointSpace':
            self.velocityCommand.setVelocity(action['jointVelocities'])

        else:
            self.velocityCommand.setVelocity(self.inverseKinematics(action))

        # publish velocity comands
        self.publishVelocity()
        
        # gripper control 
        if 'gripperRight' in action:
            self.gripperControl.setGripperPosition(gripperRight=action['gripperRight'])
        if 'gripperLeft' in action:
            self.gripperControl.setGripperPosition(gripperLeft=action['gripperLeft'])

    def inverseKinematics(self, action):
        # calculated the combined geometric jacobian from base frame to tip of gripper for both arms
        jacobianCombined = utils.CalcJacobianCombined(data=self.data.jacobian[0], \
                                                    gripperLocalTransform=self.tfFrames,\
                                                    transformer=self.transformer,\
                                                    yumiGrippPoseR=self.yumiGrippPoseR,\
                                                    yumiGrippPoseL=self.yumiGrippPoseL)        

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
        dataNP = np.asarray(self.data.jacobian[1].data)
        jacobianRightElbow = dataNP[0::2].reshape((6,4))
        jacobianLeftElbow = dataNP[1::2].reshape((6,4))
      
        self.selfCollisionElbow.compute(jacobianRightElbow=jacobianRightElbow,\
                                        jacobianLeftElbow=jacobianLeftElbow,\
                                        yumiElbowPoseR=self.yumiElbowPoseR,\
                                        yumiElbowPoseL=self.yumiElbowPoseL)
        SoT.append(self.selfCollisionElbow)
   
        self.lock.acquire() # mutex lock
        if  action['controlSpace'] == 'individual':
            # Gripper colision avoidance 
            if action['colisionAvoidance']:
                self.endeEffectorCollision.compute(jacobian=jacobianCombined,\
                                                    yumiPoseR=self.yumiGrippPoseR,\
                                                    yumiPoseL=self.yumiGrippPoseL)
                SoT.append(self.endeEffectorCollision)

            # Individual control objectiov 
            self.indiviualControl.compute(controlVelocity=action['cartesianVelocity'], jacobian=jacobianCombined)
            SoT.append(self.indiviualControl)

        elif action['controlSpace'] == 'coordinated':
            # Relative motion
            self.relativeControl.compute(controlVelocity=action['relativeVelocity'],\
                                        jacobian=jacobianCombined,\
                                        transformer=self.transformer,\
                                        yumiGripperPoseR=self.yumiGrippPoseR,\
                                        yumiGripperPoseL=self.yumiGrippPoseL)
            SoT.append(self.relativeControl) 
            # Absolute motion 
            self.absoluteControl.compute(controlVelocity=action['absoluteVelocity'],\
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

        # Joint potential task, tries to keep the robot in a good configuration 
        self.jointPositionPotential.compute(jointState=self.jointState)
        SoT.append(self.jointPositionPotential)
        
        # solve HQP
        # ----------------------
        return self.HQP.solve(SoT=SoT)
 

    def resetPose(self):
        pass

    def publishVelocity(self):
        # sends the control comands to the driver 
        msg = Float64MultiArray()
        msg.data = self.velocityCommand.getVelocityPublish()
        self.pub.publish(msg)
        """
        # sends information about which part of the trajectort is beeing executed 
        msgSubTask = Int64()
        msgSubTask.data = self.controlTarget.trajectory.index - 1 # TODO implement this in controltarget
        self.pubSubTask.publish(msgSubTask)
        """
        # send frame transformes to tf tree
        self.tfFrames.tfBroadcast()

    #

    def policy(self):
        raise NotImplementedError()







