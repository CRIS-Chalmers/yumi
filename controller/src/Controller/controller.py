#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from controller.msg import Kinematics_msg
import numpy as np
import tf
import threading
# import rosservice
from abb_robot_msgs.srv import TriggerWithResultCode
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import HQPSolver
import utils 
import Task
from parameters import Parameters


class YumiController(object):
    """Class for controlling YuMi, inherit this class and
    create your own policy() function. The policy function has to call the setAction() function"""
    def __init__(self):
        self.jointState = utils.JointState()  # keeps track of joint positions
        self.velocityCommand = utils.VelocityCommand()

        # Forward kinematics and transformers 
        # for critical updates
        self.yumiGripPoseR = utils.FramePose()
        self.yumiGripPoseL = utils.FramePose()
        self.yumiElbowPoseR = utils.FramePose()
        self.yumiElbowPoseL = utils.FramePose()

        # setup transformations 
        
        self.tfFrames = utils.TfBroadcastFrames(Parameters.gripperRightLocal,
                                                Parameters.gripperLeftLocal,
                                                Parameters.yumiToWorldLocal)
        self.tfFrames.tfBroadcast()
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

        # mutex
        self.lock = threading.Lock()
        # stack of tasks
        self.data = '_'
        # Class instance for gripper control
        self.gripperControl = utils.GripperControl() 
        self.setupTasks()

        # publish velocity commands
        self.pub = rospy.Publisher('/yumi/egm/joint_group_velocity_controller/command', Float64MultiArray,
                                   queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/Jacobian_R_L", Kinematics_msg, self.callback, queue_size=3, tcp_nodelay=True)

        # reset
        self.finalTime = 2
        self.startJointPosition = np.zeros(14) 
        self.resetting = False
        self.resetTime = 0
        # mutex
        self.lock = threading.Lock()

    def setupTasks(self):
        """ Sets up tasks for the HQP solver"""
        # HQP solver 
        stopEGM = rospy.ServiceProxy('/yumi/rws/sm_addin/stop_egm', TriggerWithResultCode)
        self.HQP = HQPSolver.HQPSolver(stopEGM)

        # Task objects -------------
        # ctype (constraint type) 0 = equality, 1 = upper, -1 = lower

        jointPositionBoundUpper = np.hstack([Parameters.jointPositionBoundUpper, Parameters.jointPositionBoundUpper])
        jointPositionBoundLower = np.hstack([Parameters.jointPositionBoundLower, Parameters.jointPositionBoundLower])

        self.jointPositionBound = Task.JointPositionBoundsTaskV2(Dof=Parameters.Dof,
                                                                 boundsUpper=jointPositionBoundUpper,
                                                                 boundsLower=jointPositionBoundLower,
                                                                 timestep=Parameters.dT)

        # joint velocity limit
        jointVelocityLimits = np.hstack([Parameters.jointVelocityBound, Parameters.jointVelocityBound])  # two arms

        self.jointVelocityBound = Task.JointVelocityBoundsTaskV2(Dof=Parameters.Dof,
                                                                 boundsUpper=jointVelocityLimits,
                                                                 boundsLower=-jointVelocityLimits)
        self.jointVelocityBound.compute()  # constant
        # end effector collision avoidance
        self.endEffectorCollision = Task.EndEffectorProximity(Dof=Parameters.Dof,
                                                              minDistance=Parameters.gripperMinDistance,
                                                              timestep=Parameters.dT)

        # Control objective
        
        self.individualControl = Task.IndividualControl(Dof=Parameters.Dof)
        self.absoluteControl = Task.AbsoluteControl(Dof=Parameters.Dof)
        self.relativeControl = Task.RelativeControl(Dof=Parameters.Dof)

        # elbow collision
        self.selfCollisionElbow = Task.ElbowProximityV2(Dof=Parameters.Dof,
                                                        minDistance=Parameters.minElbowDistance,
                                                        timestep=Parameters.dT)
        # joint potential 
        self.jointPositionPotential = Task.JointPositionPotential(Dof=Parameters.Dof,
                                                                  defaultPose=Parameters.neutralPose,
                                                                  timestep=Parameters.dT)

    def callback(self, data):
        """ Start of control loop, gets called by kdl_kinematics.cpp"""
        self.lock.acquire()
        self.data = data
        # update joint position
        self.jointState.UpdatePose(pose=np.asarray(self.data.jointPosition))
        self.jointState.UpdateVelocity(velocity=np.asarray(self.data.jointVelocity))

        # Forward kinematics, Using KDL here instead of tf tree 
        self.yumiGripPoseR.update_(self.data.forwardKinematics[0], self.transformer, self.tfFrames.getGripperRight())
        self.yumiGripPoseL.update_(self.data.forwardKinematics[1], self.transformer, self.tfFrames.getGripperLeft())
        
        self.yumiElbowPoseL.update_(self.data.forwardKinematics[3])
        self.yumiElbowPoseR.update_(self.data.forwardKinematics[2])
        self.policy()
        # send frame transforms to tf tree
        self.tfFrames.tfBroadcast()
        self.lock.release()

    def setAction(self, action):
        """ Sets and action and controls the robot, called from the self.policy() function.
        :param action: dict(), this input parameter has to contain a certain key words.
        :key action['controlSpace']: determines which control mode {jointSpace, individual
         or coordinated}
         :key action['jointVelocities']: np.array([rightArm, leftArm]) shape(14) with joint
         velocities (rad/s) (needed for mode 'jointSpace')
         :key action['cartesianVelocity']: np.array([rightT, rightR, leftT, leftR]) shape(12)
         with cartesian velocities (m/s, rad/s) (needed for mode 'individual')
         :key action['relativeVelocity']: np.array([relativeT, relativeR]) shape(6)
         with cartesian velocities in absolute frame (m/s, rad/s) (needed for mode 'coordinated')
         :key action['absoluteVelocity']: np.array([absoluteT, absoluteR]) shape(6)
         with cartesian velocities in yumi base frame (m/s, rad/s) (needed for mode 'coordinated')
         :key action['gripperRight']: float for gripper position (mm)
         :key action['gripperLeft']: float for gripper position (mm)
         For more information see examples or documentation.
         """

        # joint control
        if action['controlSpace'] == 'jointSpace':
            self.velocityCommand.setVelocity(action['jointVelocities'])
        else:
            self.velocityCommand.setVelocity(self.inverseKinematics(action))
        # publish velocity commands
        self.publishVelocity()
        # gripper control
        if 'gripperRight' in action:
            self.gripperControl.setGripperPosition(gripperRight=action['gripperRight'])
        if 'gripperLeft' in action:
            self.gripperControl.setGripperPosition(gripperLeft=action['gripperLeft'])

    def inverseKinematics(self, action):
        """Sets up stack of tasks and solves the inverse kinematics problem for
        individual or coordinated manipulaiton"""
        for key, value in Parameters.feasibilityObjectives.items():
            if key not in action:
                action[key] = value

        # calculated the combined Jacobian from base frame to tip of gripper for both arms
        jacobianCombined = utils.CalcJacobianCombined(data=self.data.jacobian[0],
                                                      gripperLocalTransform=self.tfFrames,
                                                      transformer=self.transformer,
                                                      yumiGripPoseR=self.yumiGripPoseR,
                                                      yumiGripPoseL=self.yumiGripPoseL)

        # stack of tasks, in descending hierarchy
        # ----------------------
        SoT = list()
        # velocity bound
        if action['JointVelocityBound']:
            SoT.append(self.jointVelocityBound)

        # position bound
        if action['jointPositionBound']:
            self.jointPositionBound.compute(jointState=self.jointState)
            SoT.append(self.jointPositionBound)

        # Elbow proximity limit task
        if action['elbowCollision']:
            dataNP = np.asarray(self.data.jacobian[1].data)
            jacobianRightElbow = dataNP[0::2].reshape((6, 4))
            jacobianLeftElbow = dataNP[1::2].reshape((6, 4))

            self.selfCollisionElbow.compute(jacobianRightElbow=jacobianRightElbow,
                                            jacobianLeftElbow=jacobianLeftElbow,
                                            yumiElbowPoseR=self.yumiElbowPoseR,
                                            yumiElbowPoseL=self.yumiElbowPoseL)
            SoT.append(self.selfCollisionElbow)
   
        if action['controlSpace'] == 'individual':
            # Gripper collision avoidance
            if action['gripperCollision']:
                self.endEffectorCollision.compute(jacobian=jacobianCombined,
                                                  yumiGripperPoseR=self.yumiGripPoseR,
                                                  yumiGripperPoseL=self.yumiGripPoseL)
                SoT.append(self.endEffectorCollision)

            # Individual control objective
            self.individualControl.compute(controlVelocity=action['cartesianVelocity'], jacobian=jacobianCombined)
            SoT.append(self.individualControl)

        elif action['controlSpace'] == 'coordinated':
            # Relative motion
            if 'relativeVelocity' in action:
                self.relativeControl.compute(controlVelocity=action['relativeVelocity'],
                                             jacobian=jacobianCombined,
                                             transformer=self.transformer,
                                             yumiGripperPoseR=self.yumiGripPoseR,
                                             yumiGripperPoseL=self.yumiGripPoseL)
                SoT.append(self.relativeControl)
            # Absolute motion
            if 'absoluteVelocity' in action:
                self.absoluteControl.compute(controlVelocity=action['absoluteVelocity'],
                                             jacobian=jacobianCombined)
                SoT.append(self.absoluteControl)

        else:
            print('Non valid control mode, stopping')
            return np.zeros(Parameters.Dof)

        # Joint potential task, tries to keep the robot in a good configuration
        if action['jointPotential']:
            self.jointPositionPotential.compute(jointState=self.jointState)
            SoT.append(self.jointPositionPotential)

        # solve HQP
        # ----------------------
        return self.HQP.solve(SoT=SoT)

    def resetPose(self, resetPos=Parameters.resetPos):
        """ Simple joint space controller for resetting the pose,
        use with care as if anything is in the way it will collide."""
        if not self.resetting:
            jointState = self.jointState.GetJointPosition()[0:14]
            diff = resetPos - jointState
            maxErrorAngle = max(diff)
            timeMax = maxErrorAngle/(3 * np.pi/180)
            self.finalTime = max(timeMax, 2) 
            self.startJointPosition = jointState 
            self.resetting = True 
            self.resetTime = 0

        self.resetTime += Parameters.dT

        if self.resetTime > self.finalTime:
            self.velocityCommand.setVelocity(np.zeros(14))
            self.publishVelocity()
            print('reset pose done')
            self.resetting = False
            return False
       
        q, dq = utils.calcPosVel(self.startJointPosition, np.zeros(14),
                                 resetPos, np.zeros(14), self.finalTime, self.resetTime)

        vel = dq + (q-self.jointState.GetJointPosition())
        self.velocityCommand.setVelocity(vel)
        # publish velocity commands
        self.publishVelocity()
        return True

    def publishVelocity(self):
        """Publishes velocity to yumi"""
        # sends the control commands to the driver
        msg = Float64MultiArray()
        msg.data = self.velocityCommand.getVelocityPublish()
        self.pub.publish(msg)


    def policy(self):
        """This function should generate velocity commands for the controller.
        There are three control modes, joint space control, individual control
        in cartesian space with yumi_base_link as reference frame, coordinated
        manipulation with absolute and relative control. All the inverse
        kinematics are solved with an HQP solver. This function has to call
        the setAction(action) function. The easily available observations (through
        more exists)are found in self.jointState, self.yumiGripPoseR and
        self.yumiGripPoseL"""
        raise NotImplementedError()

