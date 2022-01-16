#!/usr/bin/env python3

import rospy
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
import numpy as np
from controller.msg import YumiPoseControll_msg, YumiPose_msg
from Controller.controller import YumiController
from Controller.controlTarget import ControlTarget
from parameters import Parameters
from std_msgs.msg import Float32

# k is the gain for vel = vel + k*error
# Gain for individual Control
K_P_I = 1  # Gain for positional error
K_O_I = 1  # Gain for angular error

# Gain for absolute control
K_P_A = 0.4  # Gain for positional error
K_O_A = 0.4  # Gain for angular error

# Gain for relative control
K_P_R = 0.5  # Gain for positional error
K_O_R = 0.5  # Gain for angular error


class Controller(YumiController):
    def __init__(self):
        super(Controller, self).__init__()
        self.initTime = rospy.Time.now().to_sec()
        self.reset = False

        self.controlTarget = ControlTarget(Parameters.dT)

        # direct controller. everithing whats is recieved by the DLO primitives
        rospy.Subscriber("yumiControllPose", YumiPoseControll_msg, self.PoseControllCallback, queue_size=1)
        self.mode = 'stop' # which mode is requested? 'reset', ' coordinated', 'individual', 'stop'
        self.targetPose = np.zeros(12)

        # return error to DLO primitive to know about current state
        self.pubCurrentPose = rospy.Publisher('YumiCurrentPose', YumiPose_msg, queue_size=1)
        self.pubPoseError = rospy.Publisher('YumiPoseError', Float32, queue_size=1)

    def policy(self):

        # forward kinematics
        rightGripperPosition = self.yumiGripPoseR.getPosition()
        rightGripperOrientation = self.yumiGripPoseR.getQuaternion()  # expressed in the base frame of yumi
        leftGripperPosition = self.yumiGripPoseL.getPosition()
        leftGripperOrientation = self.yumiGripPoseL.getQuaternion()  # expressed in the base frame of yumi

        self.pubCurrentPose.publish(rightGripperPosition, rightGripperOrientation, leftGripperPosition, leftGripperOrientation)

        # A way to rest yuMi:s pose, use with care as there is no collision avoidance in the implemented function.
        # If a better reset function is desired, create your own rest function.
        if self.mode == 'reset':
            self.reset = self.resetPose() # resets pose, only needs to be set once.


        elif self.mode == 'individual':
            action = dict()  # used to store the desired action
        
            self.controlTarget.updatePose(yumiGripPoseR=self.yumiGripPoseR,
                                        yumiGripPoseL=self.yumiGripPoseL)

            targetPosition = np.array(self.targetPose[0:6])
            targetOrientation = np.array(self.targetPose[6:14])


            # set target velocities and positions
            self.controlTarget.setTarget( targetPosition, targetOrientation, np.zeros(12))

            # sets the control mode
            action['controlSpace'] = 'individual'

            # k is the gain for vel = vel + k*error
            action['cartesianVelocity'] = self.controlTarget.getIndividualTargetVelocity(k_p=K_P_I,  k_o=K_O_I)

            self.pubPoseError.publish(np.sum(np.abs(action['cartesianVelocity'])))
            
            self.setAction(action)


        elif self.mode == 'coordinated':
            action = dict()  # used to store the desired action
        
            self.controlTarget.updatePose(yumiGripPoseR=self.yumiGripPoseR,
                                        yumiGripPoseL=self.yumiGripPoseL)

            targetPosition = np.array(self.targetPose[0:6])
            targetOrientation = np.array(self.targetPose[6:14])

            # set target velocities and positions
            self.controlTarget.setTarget( targetPosition, targetOrientation, np.zeros(12))

            # sets the control mode
            action['controlSpace'] = 'coordinated'

            # k is the gain for vel = vel + k*error
            action['absoluteVelocity'] = self.controlTarget.getAbsoluteTargetVelocity(k_p=K_P_A, k_o=K_O_A)
            action['relativeVelocity'] = self.controlTarget.getRelativeTargetVelocity(k_p=K_P_R, k_o=K_O_R)

            self.pubPoseError.publish(np.sum(np.abs(action['relativeVelocity'])) + np.sum(np.abs(action['absoluteVelocity'])))

            self.setAction(action)


        # stop controller
        else:
            action = dict()
            action['controlSpace'] = 'individual'  # set the control space
            action['cartesianVelocity'] = np.zeros(12)  # set the velocity commands
            self.setAction(action)  # send them to the controller to be executed
            self.pubPoseError.publish(1)
            


    def PoseControllCallback(self, data):
            
            if len(data.targetPose) == 14:
                self.targetPose = data.targetPose
                self.mode = data.mode
            else:
                rospy.logerr("%3.1f is not a valid number of dicet control velocities! 14 needed!", len(data.targetPose))
                self.mode = 'stop'




def main():
    # starting ROS node and subscribers
    rospy.init_node('trajectoryController', anonymous=True)

    ymuiContoller = Controller()

    rospy.spin()


if __name__ == '__main__':
    main()
