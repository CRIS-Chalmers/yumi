#!/usr/bin/env python3

import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
import numpy as np


class Controller(YumiController):
    def __init__(self):
        super(Controller, self).__init__()
        self.initTime = rospy.Time.now().to_sec()
        self.reset = False

    def policy(self):
        """Replace with your own more sophisticated controller, the provided template controllers in this file
        have no feedback and only serves as a demonstration, should not be run on a real robot. For a more advanced
        controller with feedback look at trajectoryControl.py"""
        t = rospy.Time.now().to_sec() - self.initTime

        # How to access yumi observations

        # joint states
        jointPos = self.jointState.GetJointPosition()  # [Right, Left] [rad]
        jointVel = self.jointState.GetJointVelocity()  # [Right, Left] [rad/s]

        # forward kinematics
        rightGripperPosition = self.yumiGripPoseR.getPosition()
        rightGripperOrientation = self.yumiGripPoseR.getQuaternion()  # expressed in the base frame of yumi
        leftGripperPosition = self.yumiGripPoseL.getPosition()
        leftGripperOrientation = self.yumiGripPoseL.getQuaternion()  # expressed in the base frame of yumi

        # A way to rest yuMi:s pose, use with care as there is no collision avoidance in the implemented function.
        # If a better reset function is desired, create your own rest function.
        if self.reset:
            self.reset = self.resetPose()
            return

        # How to create simple joint space controller
        if t < 7:
            action = dict()
            action['controlSpace'] = 'jointSpace'  # set the control space
            # create the joint velocity commands, [right arm, left arm], [rad/s]
            vel = np.zeros(14)
            if t < 3:
                vel[3] = 0.1  # set velocity on joint 4 on both arms
                vel[10] = -0.1
            elif t < 6:
                vel[3] = -0.1
                vel[10] = 0.1
            action['jointVelocities'] = vel  # set the velocity commands
            self.setAction(action)  # send them to the controller to be executed

        # How to create simple cartesian controller where each arm is controlled individually
        elif t < 14:
            action = dict()
            action['controlSpace'] = 'individual'  # set the control space
            # create the cartesian velocity commands (expressed in the base frame),
            # [rightEndTranslation [m/s], rightEndRotation[rad/s], leftEndTranslation, leftEndRotation]
            # (x,y,z)
            vel = np.zeros(12)
            if t < 10:
                vel[1] = 0.05  # right arm 0.05 m/s in y-axis
                vel[9] = 0.1  # left arm rotate 0.1 rad/s around x-axis
            elif t < 13:
                vel[1] = -0.05
                vel[9] = -0.1
            action['cartesianVelocity'] = vel  # set the velocity commands
            self.setAction(action)  # send them to the controller to be executed

        # How to create simple cartesian controller where each arm is controlled individually
        elif t < 31:
            action = dict()
            action['controlSpace'] = 'coordinated'  # set the control space
            # absolute motion [averageTranslation [m/s], averageRotation[rad/s]]
            # relative motion [relativeTranslation[m/s], relativeRotation[rad/s]]
            # (x,y,z)
            velAbs = np.zeros(6)
            velRel = np.zeros(6)
            if t < 17:
                velAbs[2] = 0.05  # both arms upp (z-axis)
            elif t < 19:
                velAbs[3] = 0.1  # both rotate around the average frame
            elif t < 21:
                velAbs[3] = -0.1
            elif t < 24:
                velAbs[2] = -0.05
            elif t < 27:
                velRel[1] = 0.05  # relative movement, average stays the same.
            elif t < 30:
                velRel[1] = -0.05

            action['relativeVelocity'] = velRel  # set the velocity commands
            action['absoluteVelocity'] = velAbs  # set the velocity commands
            self.setAction(action)  # send them to the controller to be executed

        elif t < 31.1:
            self.reset = True  # resets pose, only needs to be set once.


def main():
    # starting ROS node and subscribers
    rospy.init_node('trajectoryController', anonymous=True)

    ymuiContoller = Controller()

    rospy.spin()


if __name__ == '__main__':
    main()
