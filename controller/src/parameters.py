#!/usr/bin/env python3

import numpy as np
import Controller.utils as utils
import tf

# This variable class stores all tunable parameters for the
# controller (only for the parts written in python) 

class Parameters():
    # Hz the controller is running, is also defined in kdl_kinematics.cpp (both needs to be the same!)
    updateRate = 50 
    dT = 1/50 
    
    # degrees of freedom for the robot (Code will not work if changed without other modifications)
    Dof = 14    
    
    # k is the gain for vel = vel + k*error
    # Gain for individual Control
    k_p_i = 2  # Gain for positional error
    k_o_i = 2  # Gain for angular error

    # Gain for absolute control 
    k_p_a = 2  # Gain for positional error
    k_o_a = 2  # Gain for angular error

    # Gain for relative control
    k_p_r = 2  # Gain for positional error
    k_o_r = 2  # Gain for angular error

    # reset position 
    resetPos = np.array([0.7, -1.7, -0.8, 1.0, -2.2, 1.0, 0.0, -0.7, -1.7, 0.8, 1.0, 2.2, 1.0, 0.0])  # need to be tuned
    #
    feasibilityObjectives = {'jointPositionBound': True,
                             'JointVelocityBound': True,
                             'elbowCollision': True,
                             'gripperCollision': False,
                             'jointPotential': True}
                             
    # Max values before joints becomes saturated, values are from
    # https://search.abb.com/library/Download.aspx?DocumentID=3HAC052982-001&LanguageCode=en&DocumentPartId=&Action=Launch
    jointPositionBoundUpper = np.array([168.5, 43.5, 168.5, 80, 290, 138, 229])*np.pi/180 * 0.99  # in radians
    jointPositionBoundLower = np.array([-168.5, -143.5, -168.5, -123.5, -290, -88, -229])*np.pi/180 * 0.99  # in radians

    # joint velocity limit [rad/s], by ABB naming it is joints [1,2,7,3,4,5,6]
    jointVelocityBound = np.array([1, 1, 1, 1, 1, 1, 1])

    # gripper collision avoidance (Only for individual motion and not coordinated motion)
    gripperMinDistance = 0.12  # closet allowed distance in [m]

    # elbow collision avoidance  
    minElbowDistance = 0.2  # closes the elbows can be to each other in [m]
    
    # For joint potential, defining a neutral pose to move towards
    neutralPose = np.array([0.7, -1.7, -0.8, 1.0, -2.2, 1.0, 0.0, -0.7, -1.7, 0.8, 1.0, 2.2, 1.0, 0.0])

    # used for coordinated manipulation only, individual can deviate due to gripper collision avoidance
    # maxTrajectoryDeviaiton = np.array([0.01,0.01,0.01, 0.1,0.1,0.1, 0.01,0.01,0.01, 0.1,0.1,0.1])
    rotLocal = tf.transformations.quaternion_from_euler(35*np.pi/180, 0, 0, 'rzyx')
    gripperRightLocal = utils.FramePose(position=np.array([0, 0, 0.143]), quaternion=rotLocal)
    gripperLeftLocal = utils.FramePose(position=np.array([0, 0, 0.143]), quaternion=np.array([0, 0, 0, 1]))
    yumiToWorldLocal = utils.FramePose(position=np.array([0.181, 0, 0]), quaternion=np.array([0, 0, 0.7071, -0.7071]))
