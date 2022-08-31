#!/usr/bin/env python3


import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
#from controller.msg import CartesianVelocityControl_msg
from controller.msg import PositionAndVelocity_msg
from std_msgs.msg import Float32MultiArray

import numpy as np


class Controller(YumiController):
    def __init__(self):
        super(Controller, self).__init__()
        self.lastTime = rospy.Time.now().to_sec()
        self.reset = False
        # create the cartesian velocity commands (expressed in the base frame),
        # [rightEndTranslation [m/s], rightEndRotation[rad/s], leftEndTranslation, leftEndRotation]
        self.velocity = np.zeros(12)

        self.rightGripper = -1
        self.leftGripper = -1
        self.newGripperLeft = False
        self.newGripperRight = False
        
        self.timeout = 0.5
        self.velocity_pub = rospy.Publisher('/yumi/egm/positionAndVelocity', PositionAndVelocity_msg, 
                                            queue_size=1, tcp_nodelay=True)

        #rospy.Subscriber("/yumi/egm/cartesianVelocityCommand", CartesianVelocityControl_msg, self.cartesianVelCallback, 
        #                 queue_size=3, tcp_nodelay=True)

        rospy.Subscriber("/yumi/egm/cartesianVelocityCommand", Float32MultiArray, self.cartesianVelCallback, 
                         queue_size=3, tcp_nodelay=True)

    def policy(self):

        t_diff = rospy.Time.now().to_sec() - self.lastTime
        if t_diff > self.timeout:
            self.velocity = np.zeros(12)

        # joint states
        jointPos = self.jointState.GetJointPosition()  # [Right, Left] [rad]
        jointVel = self.jointState.GetJointVelocity()  # [Right, Left] [rad/s]

        # forward kinematics
        rightGripperPosition = self.yumiGripPoseR.getPosition()
        rightGripperOrientation = self.yumiGripPoseR.getQuaternion()  # expressed in the base frame of yumi
        leftGripperPosition = self.yumiGripPoseL.getPosition()
        leftGripperOrientation = self.yumiGripPoseL.getQuaternion()  # expressed in the base frame of yumi

        action = dict()
        action['controlSpace'] = 'individual'  # set the control space
        
        cartesianVel = self.jacobianCombined.dot(jointVel)

        action['cartesianVelocity'] = self.velocity  # set the velocity commands

        if self.newGripperRight:
            action['gripperRight'] = self.rightGripper
            self.newGripperRight = False
        if self.newGripperLeft:
            action['gripperLeft'] = self.leftGripper
            self.newGripperLeft = False

        self.setAction(action)  # send them to the controller to be executed

        msg = PositionAndVelocity_msg()
        msg.header.stamp = rospy.Time.now()

        msg.positionLeft = leftGripperPosition
        msg.positionRight = rightGripperPosition
        msg.orientationLeft = leftGripperOrientation
        msg.orientationRight = rightGripperOrientation
        
        msg.linearVelocityLeft = cartesianVel[6:9]
        msg.linearVelocityRight = cartesianVel[0:3]
        msg.angularVelocityLeft = cartesianVel[9:12]
        msg.angularVelocityRight = cartesianVel[3:6]

        self.velocity_pub.publish(msg)

    def cartesianVelCallback(self, data):
        command_list = data.data
        #linLeft = data.linearVelocityLeft
        #linRight = data.linearVelocityRight
        #angLeft = data.angularVelocityLeft
        #angRight = data.angularVelocityRight
        linLeft = command_list[0:3]
        linRight = command_list[3:6] 
        angLeft = command_list[6:9]
        angRight = command_list[9:12]
        gripperLeft =command_list[12]
        gripperRight =command_list[13]

        self.velocity = np.hstack([linRight, angRight, linLeft, angLeft])

        self.lastTime = rospy.Time.now().to_sec()
        '''
        if data.gripperLeft != self.leftGripper:
            self.leftGripper = data.gripperLeft
            self.newGripperLeft = True

        if data.gripperRight != self.rightGripper:
            self.rightGripper = data.gripperRight
            self.newGripperRight = True
        '''
        if gripperLeft != self.leftGripper:
            self.leftGripper = gripperLeft
            self.newGripperLeft = True

        if gripperRight != self.rightGripper:
            self.rightGripper = gripperRight
            self.newGripperRight = True
        

def main():
    # starting ROS node and subscribers
    rospy.init_node('yumiCartesianVelocityController', anonymous=True)

    ymuiContoller = Controller()

    rospy.spin()


if __name__ == '__main__':
    main()