#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np

def main():

    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True) 
    pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
    rospy.sleep(0.1)

    # --------------------------------------------------

    msg = Trajectory_msg() # message will contain list of trajectory points
    msg.header.stamp = rospy.Time.now() 
    msg.mode = 'individual' # control mode

    rotLeft = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 90*np.pi/180, 'rzyx')
    rotRight = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, -90*np.pi/180, 'rzyx')

    # ---------------
    trajectoryPoint = Trajectory_point() # point 
    trajectoryPoint.positionRight = [0.35, -0.2, 0.2] # poition right arm [m], yumi_base_link is the origin 
    trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]# poition left arm [m]
    trajectoryPoint.orientationLeft = [1,0,0,0] # orientation left arm, quaterniorns [x, y, z, w]
    trajectoryPoint.orientationRight = [1,0,0,0]# orientation right arm
    trajectoryPoint.gripperLeft = 20.0 # gripper width for the fingers [mm]
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 12.0 # time to get to this point [s]

    trajectory = [trajectoryPoint]
    
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, 0, 0.35]
    trajectoryPoint.positionLeft = [0.05, 0.35, 0.10]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = 0
    trajectoryPoint.gripperRight = 0
    trajectoryPoint.pointTime = 10.0

    trajectory.append(trajectoryPoint)
    
    msg.trajectory = trajectory
    
    pub.publish(msg)
    rospy.sleep(0.1)
    pub.publish(msg)

    
    rospy.spin()


if __name__ == '__main__':
    main()


