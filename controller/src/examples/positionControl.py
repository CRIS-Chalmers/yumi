#!/usr/bin/env python3


import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
from Controller.controlTarget import ControlTarget
from parameters import Parameters
from controller.msg import Trajectory_msg
from std_msgs.msg import Float32MultiArray
import Controller.utils as utils
import numpy as np
from std_msgs.msg import Int64
import threading

# k is the gain for vel = vel + k*error
# Gain for individual Control
K_P_I = 2  # Gain for positional error
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
        self.cartesianVel = rospy.Publisher('/yumi/egm/endeffectoVelCartesian', Float32MultiArray, queue_size=1)

        self.lockTrajectory = threading.Lock()
        self.maxDeviation = np.array([0.015, 0.25, 0.015, 0.25])

    def policy(self):
        """Gets called for each time step and calculates the target velocity"""
        
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

        jointVel = self.jointState.GetJointVelocity()  # [Right, Left] [rad/s]
        cartesianVel = self.jacobianCombined.dot(jointVel)
        cartesianVel_m = np.hstack([cartesianVel[6:9], cartesianVel[0:3], cartesianVel[9:12], cartesianVel[3:6]]).tolist()


        jointVel = self.jointState.GetJointVelocity()  # [Right, Left] [rad/s]
        cartesianVel = self.jacobianCombined.dot(jointVel)
        cartesianVel_m = np.hstack([cartesianVel[6:9], cartesianVel[0:3], cartesianVel[9:12], cartesianVel[3:6]]).tolist()

        msgVel = Float32MultiArray()
        msgVel.data = cartesianVel_m
        self.cartesianVel.publish(msgVel)



    def callbackTrajectory(self, data):
        """ Gets called when a new set of trajectory parameters is received
         The variable names in this function and the the trajectory class follows
         individual motion with left and right . This means when coordinate manipulation
         is usd, right is absolute motion and left becomes relative motion. """        

        point = data.data

        pointLPos = point[0:3]
        pointRPos = point[3:6]
        pointLQuat = point[6:10]
        pointRQuat = point[10:14]

        gripper_L = point[14]
        gripper_R = point[15]

        point_time = point[16]

        if not self.controlTarget.dataReceived:
            print('No data received, start robot or simulation before sending trajectory')
            return
        # get current pose, used as first trajectory parameter

        positionRight = np.copy(self.controlTarget.translationRightArm)
        positionLeft  = np.copy(self.controlTarget.translationLeftArm)
        orientationRight = np.copy(self.controlTarget.rotationRightArm)
        orientationLeft = np.copy(self.controlTarget.rotationLeftArm)
        self.reset = False

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


        positionRight = np.asarray(pointRPos)
        positionLeft  = np.asarray(pointLPos)
        orientationRight = np.asarray(pointRQuat)
        orientationLeft = np.asarray(pointLQuat)

        gripperLeft = gripper_L
        gripperRight = gripper_R
        pointTime = np.asarray(point_time)
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
        elif self.controlTarget.mode == 'individual':  # no change in control mode
            velLeftInit = np.copy(self.controlTarget.targetVelocities[6:9])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)

        self.lockTrajectory.acquire()
        # set mode
        self.controlTarget.mode = 'individual'

        # update the trajectory
        self.controlTarget.trajectory.updateTrajectory(trajectory, velLeftInit, velRightInit)
        self.controlTarget.trajIndex = 0
        self.controlTarget.trajectorySegment = 0
        self.lockTrajectory.release()


def main():
    """main function"""
    # starting ROS node and subscribers
    rospy.init_node('positionControl', anonymous=True) 

    ymuiContoller = TrajectoryController()
    rospy.Subscriber("/yumi/egm/position", Float32MultiArray, ymuiContoller.callbackTrajectory, queue_size=1, tcp_nodelay=True)

    rospy.spin()

if __name__ == '__main__':
    main()
