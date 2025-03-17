#!/usr/bin/env python3


import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
from Controller.controlTarget import ControlTarget
from parameters import Parameters
from yumi_controller.msg import Trajectory_msg
from abb_robot_msgs.msg import SystemState
from abb_robot_msgs.srv import TriggerWithResultCode

import Controller.utils as utils
import numpy as np
from std_msgs.msg import Int64
import threading

# k is the gain for vel = vel + k*error
# Gain for individual Control
K_P_I = 4  # Gain for positional error
K_O_I = 2  # Gain for angular error

# Gain for absolute control
K_P_A = 3  # Gain for positional error
K_O_A = 3  # Gain for angular error

# Gain for relative control
K_P_R = 4  # Gain for positional error
K_O_R = 4  # Gain for angular error

class TrajectoryController(YumiController):
    """Class for running trajectory control, trajectory parameters are sent with ros and
    from those a trajectory is constructed and followed."""
    def __init__(self):
        super(TrajectoryController, self).__init__()
        self.controlTarget = ControlTarget(Parameters.dT)
        self.reset = False
        self.pubSubTask = rospy.Publisher('/controller/sub_task', Int64, queue_size=1)
        self.lockTrajectory = threading.Lock()
        self.maxDeviation = np.array([0.015, 0.25, 0.015, 0.25])*2
        self.auto_mode = True
        rospy.Subscriber("/yumi/rws/system_states", SystemState, self.callback_system_state, queue_size=3, tcp_nodelay=True)
        self.restart_rapid = rospy.ServiceProxy('/yumi/rws/start_rapid', TriggerWithResultCode)

    def callback_system_state(self, data):
        auto_mode = data.auto_mode

        if not self.auto_mode and auto_mode:
            self.reset_trajectory()
            self.restart_rapid()
            print('trajectory reset')

        self.auto_mode = auto_mode

    def reset_trajectory(self):
        self.lockTrajectory.acquire()
        self.controlTarget = ControlTarget(Parameters.dT)
        self.lockTrajectory.release()

    def policy(self):
        """Gets called for each time step and calculates the target velocity"""
        if self.auto_mode == False:
            action = dict()  # used to store the desired action
            action['controlSpace'] = 'jointSpace'
            action['jointVelocities'] = np.zeros(Parameters.Dof)
            #print('control loop off')
            return

        # Update the pose for controlTarget class
        self.lockTrajectory.acquire()

        self.controlTarget.updatePose(yumiGripPoseR=self.yumiGripPoseR,
                                      yumiGripPoseL=self.yumiGripPoseL)
        # resets yumi to init pose
        if self.reset:
            self.reset = self.resetPose()
            if self.reset == False:
                self.controlTarget = ControlTarget(Parameters.dT)
            self.lockTrajectory.release()
            return
            
        action = dict()  # used to store the desired action


        # calculates target velocities and positions
        self.controlTarget.updateTarget()

        # send commands to the grippers
        if self.controlTarget.checkNewTrajectorySegment():
            action['gripperRight'] = self.controlTarget.gripperRight
            action['gripperLeft'] = self.controlTarget.gripperLeft

        # sets the control mode
        action['controlSpace'] = self.controlTarget.mode

        # k is the gain for vel = vel + k*error
        if self.controlTarget.mode == 'individual':
            action['cartesianVelocity'] = self.controlTarget.getIndividualTargetVelocity(k_p=K_P_I,
                                                                                         k_o=K_O_I)
        elif self.controlTarget.mode == 'coordinated':
            action['absoluteVelocity'] = self.controlTarget.getAbsoluteTargetVelocity(k_p=K_P_A,
                                                                                      k_o=K_O_A)
            action['relativeVelocity'] = self.controlTarget.getRelativeTargetVelocity(k_p=K_P_R,
                                                                                      k_o=K_O_R)

        # check so deviation from the trajectory is not too big, stop if it is
        # (turned of if gripperCollision is active for individual mode)
        if self.controlTarget.checkTrajectoryDeviation(self.maxDeviation):
            print('Deviation from trajectory too large, stopping')
            action['controlSpace'] = 'jointSpace'
            action['jointVelocities'] = np.zeros(Parameters.Dof)

        self.setAction(action)

        # sends information about which part of the trajectory is being executed
        msgSubTask = Int64()
        msgSubTask.data = self.controlTarget.trajectory.index - 1
        self.lockTrajectory.release()
        self.pubSubTask.publish(msgSubTask)

    def callbackTrajectory(self, data):
        """ Gets called when a new set of trajectory parameters is received
         The variable names in this function and the the trajectory class follows
         individual motion with left and right . This means when coordinate manipulation
         is usd, right is absolute motion and left becomes relative motion. """

        if not self.controlTarget.dataReceived:
            print('No data received, start robot or simulation before sending trajectory')
            return
        # get current pose, used as first trajectory parameter
        if data.mode == 'coordinated':
            positionRight = np.copy(self.controlTarget.absolutePosition)
            positionLeft  = np.copy(self.controlTarget.relativePosition)
            orientationRight = np.copy(self.controlTarget.absoluteOrientation)
            orientationLeft = np.copy(self.controlTarget.rotationRelative)
            self.reset = False
        elif data.mode == 'individual':
            positionRight = np.copy(self.controlTarget.translationRightArm)
            positionLeft  = np.copy(self.controlTarget.translationLeftArm)
            orientationRight = np.copy(self.controlTarget.rotationRightArm)
            orientationLeft = np.copy(self.controlTarget.rotationLeftArm)
            self.reset = False
        elif data.mode == 'resetPose':
            self.reset = True
            return
        else:
            print('Error, mode not matching combined or individual')
            return
        # current gripper position 
        gripperLeft = self.controlTarget.gripperLeft
        gripperRight = self.controlTarget.gripperRight
        # add current state as a trajectory point
        currentPoint = utils.TrajectoryPoint(positionRight=positionRight,
                                                    positionLeft=positionLeft,
                                                    orientationRight=orientationRight,
                                                    orientationLeft=orientationLeft,
                                                    gripperLeft=gripperLeft,
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
            trajectroyPoint = utils.TrajectoryPoint(positionRight=positionRight,
                                                    positionLeft=positionLeft,
                                                    orientationRight=orientationRight,
                                                    orientationLeft=orientationLeft,
                                                    gripperLeft=gripperLeft,
                                                    gripperRight=gripperRight,
                                                    pointTime=pointTime)
            trajectory.append(trajectroyPoint)

        # use current velocity for smoother transitions, (only for translation)
        if self.controlTarget.checkTrajectoryDeviation(self.maxDeviation):
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)
        elif self.controlTarget.mode == data.mode:  # no change in control mode
            velLeftInit = np.copy(self.controlTarget.targetVelocities[6:9])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        elif self.controlTarget.mode == 'individual':  # going from individual to coordinate motion
            # simple solution, not fully accurate transition
            velLeftInit = np.zeros(3)
            velRightInit = 0.5*(np.copy(self.controlTarget.targetVelocities[0:3]) +
                                np.copy(self.controlTarget.targetVelocities[6:9]))
        elif self.controlTarget.mode == 'coordinated':  # going from coordinated to individual motion
            # simple solution, not fully accurate transition
            velLeftInit = np.copy(self.controlTarget.targetVelocities[0:3])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)

        self.lockTrajectory.acquire()
        # set mode
        self.controlTarget.mode = data.mode

        # update the trajectory
        self.controlTarget.trajectory.updateTrajectory(trajectory, velLeftInit, velRightInit)
        self.controlTarget.trajIndex = 0
        self.controlTarget.trajectorySegment = 0
        self.lockTrajectory.release()


def main():
    """main function"""
    # starting ROS node and subscribers
    rospy.init_node('trajectoryController', anonymous=True) 

    ymuiContoller = TrajectoryController()
    rospy.Subscriber("/Trajectroy", Trajectory_msg, ymuiContoller.callbackTrajectory, queue_size=1, tcp_nodelay=True)

    rospy.spin()

if __name__ == '__main__':
    main()