#!/usr/bin/env python3

import numpy as np
import rospy

class JointState(object):
    def __init__(self,\
            jointPosition=np.array([1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0]),\
            jointVelocity=np.zeros(14),\
            gripperRightPosition=np.zeros(2),\
            gripperLeftPosition=np.zeros(2),\
            gripperRightVelocity=np.zeros(2),\
            gripperLeftVelocity=np.zeros(2)):

        self.jointPosition = jointPosition # only arm not gripper
        self.jointVelocity = jointVelocity # only arm not gripper
        self.gripperRightPosition = gripperRightPosition
        self.gripperLeftPosition = gripperLeftPosition
        self.gripperRightVelocity = gripperRightVelocity
        self.gripperLeftVelocity = gripperLeftVelocity
    
    def GetJointVelocity(self):
        return np.hstack([self.jointVelocity, \
            self.gripperRightVelocity, self.gripperLeftVelocity])
    
    def GetJointPosition(self):
        return np.hstack([self.jointPosition, \
            self.gripperRightPosition, self.gripperLeftPosition])

    def UpdatePose(self, pose):
        self.jointPosition = pose[0:14]
        self.gripperRightPosition = pose[14:16]
        self.gripperLeftPosition = pose[16:18]

    def UpdateVelocity(self, vel):
        self.jointVelocity = vel[0:14]
        self.gripperRightVelocity = vel[14:16]
        self.gripperLeftVelocity = vel[16:18]
