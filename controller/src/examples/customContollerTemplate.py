#!/usr/bin/env python3

import rospy

import os, sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
from Controller.controlTarget import ControlTarget
from parameters import Parameters
from controller.msg import Trajectory_msg
import Controller.utils as utils
import numpy as np
from std_msgs.msg import Int64


class TrajectoryController(YumiController):
    def __init__(self):
        super(TrajectoryController, self).__init__()


    def policy(self):

        action = dict()

        action['controlSpace'] = 'individual'
        vel = np.zeros(12)
        vel[3] = 0.1
        action['cartesianVelocity'] = vel

        self.setAction(action)




def main():
    # starting ROS node and subscribers
    rospy.init_node('trajectoryController', anonymous=True)

    ymuiContoller = TrajectoryController()

    rospy.spin()


if __name__ == '__main__':
    main()