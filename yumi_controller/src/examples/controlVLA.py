#!/usr/bin/env python3

import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
import numpy as np
import Controller.utils as utils
from yumi_controller.msg import DeltaVLA_msg
from yumi_controller.msg import PositionAndVelocity_msg

class Controller(YumiController):
    def __init__(self):
        super(Controller, self).__init__()
        self.lastTime = rospy.Time.now().to_sec()
        self.reset = False
        # create the cartesian velocity commands (expressed in the base frame),
        # [rightEndTranslation [m/s], rightEndRotation[rad/s], leftEndTranslation, leftEndRotation]
        self.velocity = np.zeros(12)


        self.rightGripper = 10
        self.leftGripper = 10
        self.newGripperLeft = False
        self.newGripperRight = False

        self.pos_controller = {'rx': HoldPosition(),
                               'ry': HoldPosition(),
                               'rz': HoldPosition(),
                               'lx': HoldPosition(),
                               'ly': HoldPosition(),
                               'lz': HoldPosition()}
        
        self.rot_controller = {'right': HoldOrientation(),
                                'left': HoldOrientation()}
        
        self.timeout = 1.0 # one second without message, then yumi will time out and stop. 

        self.velocity_pub = rospy.Publisher('/yumi/egm/positionAndVelocity', PositionAndVelocity_msg, 
                                            queue_size=1, tcp_nodelay=True)
        
        rospy.Subscriber("/yumi/egm/deltaVLACommand", DeltaVLA_msg, self.deltaVelCallback, 
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

        # A way to rest yuMi:s pose, use with care as there is no collision avoidance in the implemented function.
        # If a better reset function is desired, create your own rest function.
        if self.reset:
            self.reset = self.resetPose()
            return

        action = dict()
        action['controlSpace'] = 'individual'  # set the control space
        # create the cartesian velocity commands (expressed in the base frame),
        # [rightEndTranslation [m/s], rightEndRotation[rad/s], leftEndTranslation, leftEndRotation]
        # (x,y,z)

        velocity = self.velocity.copy() 

        velocity[0] = self.pos_controller['rx'].step(velocity[0], rightGripperPosition[0])
        velocity[1] = self.pos_controller['ry'].step(velocity[1], rightGripperPosition[1])
        velocity[2] = self.pos_controller['rz'].step(velocity[2], rightGripperPosition[2])
        velocity[3:6] = self.rot_controller['right'].step(velocity[3:6], rightGripperOrientation)
        # velocity[3:6] = 2*utils.RotationError(rightGripperOrientation, np.array([1, 0, 0, 0]) , 0.2)

        velocity[6] = self.pos_controller['lx'].step(velocity[6], leftGripperPosition[0])
        velocity[7] = self.pos_controller['ly'].step(velocity[7], leftGripperPosition[1])
        velocity[8] = self.pos_controller['lz'].step(velocity[8], leftGripperPosition[2])
        velocity[9:12] = self.rot_controller['left'].step(velocity[9:12], leftGripperOrientation)

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

        cartesianVel = self.jacobianCombined.dot(jointVel)

        msg.linearVelocityLeft = cartesianVel[6:9]
        msg.linearVelocityRight = cartesianVel[0:3]
        msg.angularVelocityLeft = cartesianVel[9:12]
        msg.angularVelocityRight = cartesianVel[3:6]

        self.velocity_pub.publish(msg)


    def deltaVelCallback(self, data):
        lin = np.zeros(3)
        ang = np.zeros(3)

        lower_limit = -1
        upper_limit = 1
        lin[0] = np.clip(data.deltaP.x, lower_limit, upper_limit) * data.max_linear_vel 
        lin[1] = np.clip(data.deltaP.y, lower_limit, upper_limit) * data.max_linear_vel
        lin[2] = np.clip(data.deltaP.z, lower_limit, upper_limit) * data.max_linear_vel
        ang[0] = np.clip(data.deltaO.x, lower_limit, upper_limit) * data.max_angular_vel
        ang[1] = np.clip(data.deltaO.y, lower_limit, upper_limit) * data.max_angular_vel
        ang[2] = np.clip(data.deltaO.z, lower_limit, upper_limit) * data.max_angular_vel
        
        if data.arm == "Left":
            self.velocity[6:] = np.hstack([lin, ang])
            if data.gripper >= 0:
                gripperLeft = 20
            else:
                gripperLeft = 0
            if gripperLeft != self.leftGripper:
                self.leftGripper = gripperLeft
                self.newGripperLeft = True
        elif data.arm == "Right":
            self.velocity[0:6] = np.hstack([lin, ang])
            if data.gripper >= 0:
                gripperRight = 20
            else:
                gripperRight = 0
            if gripperRight != self.rightGripper:
                self.rightGripper = gripperRight
                self.newGripperRight = True
        else:
            self.velocity = np.zeros(12)

        self.lastTime = rospy.Time.now().to_sec()



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


class HoldOrientation(object):
    def __init__(self):
        self.target_orientation = np.array([1, 0, 0, 0])
        self.non_zero_rot = np.zeros(3)
        self.init_bool = True

    def step(self, rot_velocity, gripper_quat):
        if self.init_bool:
            self.target_orientation = gripper_quat
            self.init_bool = False
        hold_vel = 2*utils.RotationError(gripper_quat, self.target_orientation, 0.2)
        vel = np.zeros(3)

        for i in range(3): 
            if rot_velocity[i] != 0:
                vel[i] = rot_velocity[i]
                self.non_zero_rot[i] = 1
            elif self.non_zero_rot[i] == 1:
                self.target_orientation = gripper_quat
                self.non_zero_rot[i] = 0
                vel[i] = hold_vel[i]
            else:
                vel[i] = hold_vel[i]

        return vel



def main():
    # starting ROS node and subscribers
    rospy.init_node('trajectoryController', anonymous=True)

    ymuiContoller = Controller()

    rospy.spin()


if __name__ == '__main__':
    main()
