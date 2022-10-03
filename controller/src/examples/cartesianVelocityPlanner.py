#!/usr/bin/env python3


import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
import utils


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
        
        self.pos_controller = {'rx': HoldPosition(),
                               'ry': HoldPosition(),
                               'rz': HoldPosition(),
                               'lx': HoldPosition(),
                               'ly': HoldPosition(),
                               'lz': HoldPosition()}
        

        self.timeout = 0.5
        self.velocity_pub = rospy.Publisher('/yumi/egm/positionAndVelocity', PositionAndVelocity_msg, 
                                            queue_size=1, tcp_nodelay=True)
        self.cartesianVel = rospy.Publisher('/yumi/egm/endeffectoVelCartesian', Float32MultiArray, queue_size=1)

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


        velocity = self.velocity.copy() 

        velocity[0] = self.pos_controller['rx'].step(velocity[0], rightGripperPosition[0])
        velocity[1] = self.pos_controller['ry'].step(velocity[1], rightGripperPosition[1])
        velocity[2] = self.pos_controller['rz'].step(velocity[2], rightGripperPosition[2])

        velocity[3:6] = 2*utils.RotationError(rightGripperOrientation, np.array([1, 0, 0, 0]) , 0.2)

        velocity[6] = self.pos_controller['lx'].step(velocity[6], leftGripperPosition[0])
        velocity[7] = self.pos_controller['ly'].step(velocity[7], leftGripperPosition[1])
        velocity[8] = self.pos_controller['lz'].step(velocity[8], leftGripperPosition[2])

        velocity[9:12] =  2*utils.RotationError(leftGripperOrientation, np.array([1, 0, 0, 0]) , 0.2)

        action['cartesianVelocity'] = velocity  # set the velocity commands

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


        cartesianVel_m = np.hstack([cartesianVel[6:9], cartesianVel[0:3], cartesianVel[9:12], cartesianVel[3:6]]).tolist()

        msgVel = Float32MultiArray()
        msgVel.data = cartesianVel_m
        self.cartesianVel.publish(msgVel)

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

        if gripperLeft != self.leftGripper:
            self.leftGripper = gripperLeft
            self.newGripperLeft = True

        if gripperRight != self.rightGripper:
            self.rightGripper = gripperRight
            self.newGripperRight = True


class HoldPosition(object):
    def __init__(self):
        self.last_vel_command = 0
        self.position_control = 1
        self.target_position = None
        self.P = 2
        self.max_vel = 0.05

    def step(self, vel_command, position):
        if vel_command == 0 and self.last_vel_command != 0:
            self.position_control = 1
            self.target_position = position
        elif vel_command != 0:
            self.position_control = 0
        
        if self.position_control == 1:
            if self.target_position is None:
                self.target_position = position
            error = self.target_position - position
            error = np.sign(error)* np.min([abs(error), self.max_vel])
            vel = self.P * error
        else:
            vel = vel_command

        self.last_vel_command = vel_command
        return vel



def main():
    # starting ROS node and subscribers
    rospy.init_node('yumiCartesianVelocityPlanner', anonymous=True)

    ymuiContoller = Controller()

    rospy.spin()


if __name__ == '__main__':
    main()