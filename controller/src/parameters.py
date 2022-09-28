

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

    # reset position 
    resetPos = np.array([0.7, -1.7, -0.8, 1.0, -2.2, 1.0, 0.0, -0.7, -1.7, 0.8, 1.0, 2.2, 1.0, 0.0])

    # Which feasibility objectives that should be included in HQP, only used for inverse-kinematics and
    # not for pure joint-space control
    feasibilityObjectives = {'jointPositionBound': True,
                             'JointVelocityBound': True,
                             'elbowCollision': True,
                             'gripperCollision': False,  # only used for individual control
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

    # set local offset between "yumi_link_7_r" and "yumi_grip_r" which becomes the frame controlled.
    #rotLocal = tf.transformations.quaternion_from_euler(35*np.pi/180, 0, 0, 'rzyx')
    gripperRightLocal = utils.FramePose(position=np.array([0, 0, 0.146]), quaternion=np.array([0, 0, 0, 1]))
    gripperLeftLocal = utils.FramePose(position=np.array([0, 0, 0.146]), quaternion=np.array([0, 0, 0, 1]))
    yumiToWorldLocal = utils.FramePose(position=np.array([0.181, 0, 0]), quaternion=np.array([0, 0, 0.7071, -0.7071]))
