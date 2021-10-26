#!/usr/bin/env python3

import numpy as np
import tf
import rospy
import os, sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from parameters import Parameters

class Task(object): # based on https://github.com/ritalaezza/sot-myo/blob/akos_re/src/Task.py
    """
    Base abstract class for all tasks.
    """
    def __init__(self, Dof, slackRatio=1e4):
        self.Dof = Dof
        self.constraintMatrix = np.array([])
        self.constraintVector = np.array([])
        self.constraintType = None  # 0 for equality, 1 for Gx <= h, -1 for -Gx >= -h
        self.slackRatio = slackRatio         # Between qdot and w cost

    def ndim(self):
        """
        Returns number of joint variables i.e. degrees of freedom in the robotic structure.
        """
        return self.Dof

    def mdim(self):
        """
        Returns the number of constraint equations defining the task.
        """
        return self.constraintVector.size

    def slack_ratio(self):
        return self.slackRatio

    def append_slack_locked(self, m, w):
        """
        Returns constraintMatrix and constraintVector for previously solved tasks including optimal slack variables,
        thus defining their nullspaces.
        """
        
        if self.constraintType == 0:
            A = np.hstack((self.constraintMatrix, np.zeros((self.mdim(), m))))
            b = self.constraintVector + w
            G = None
            h = None
        elif self.constraintType == 1:
            A = None
            b = None
            G = np.hstack((self.constraintMatrix, np.zeros((self.mdim(), m))))
            h = self.constraintVector + np.maximum(w, 0)
        elif self.constraintType == -1:
            A = None
            b = None
            G = np.hstack((self.constraintMatrix, np.zeros((self.mdim(), m))))
            h = self.constraintVector - np.maximum(w, 0)
           
        return A, b, G, h

    def append_slack(self, m):
        """
        Returns constraintMatrix and constraintVector with m added slack variables, i.e. one for each row of the task.
        """
        if self.constraintType == 0:
            A = np.hstack((self.constraintMatrix, -np.eye(m)))
            b = self.constraintVector
            G = np.zeros((1, A.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            h = np.zeros((1, ))                 # Does not do anything otherwise.
            #G = None
            #h = None
        elif self.constraintType == 1:
            G = np.hstack((self.constraintMatrix, -np.eye(m)))
            h = self.constraintVector
            A = np.zeros((1, G.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            b = np.zeros((1, ))                 # Does not do anything otherwise.
            #A = None
            #b = None
        elif self.constraintType == -1: # think this might have the wrong sign on the relaxation variable
            G = np.hstack((self.constraintMatrix, np.eye(m)))
            h = self.constraintVector
            A = np.zeros((1, G.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            b = np.zeros((1, ))                 # Does not do anything otherwise.
            #A = None
            #b = None
        return A, b, G, h
    


class JointPositionBoundsTaskV2(Task):
    # Task for keeping joint poisions from sturating 
    def __init__(self, Dof, boundsUpper, boundsLower, timestep):
        super(JointPositionBoundsTaskV2, self).__init__(Dof, 1e3)
        self.timestep = timestep
        self.boundsUpper = boundsUpper
        self.boundsLower = boundsLower
        self.constraintType = 1

    def compute(self, jointState):
            constraintMatrixUpper = self.timestep * np.eye(self.ndim())
            constraintVectorUpper = self.boundsUpper - jointState.jointPosition

            constraintMatrixLower = -self.timestep * np.eye(self.ndim())
            constraintVectorLower = -self.boundsLower + jointState.jointPosition  

            self.constraintMatrix = np.vstack((constraintMatrixUpper,constraintMatrixLower))
            self.constraintVector = np.hstack((constraintVectorUpper,constraintVectorLower))



class JointVelocityBoundsTaskV2(Task):
    # Task for keeping joint velocity within limits
    def __init__(self, Dof, boundsUpper, boundsLower):
        super(JointVelocityBoundsTaskV2, self).__init__(Dof, 1e3)
        self.boundsUpper = boundsUpper
        self.boundsLower = boundsLower
        self.constraintType = 1

    def compute(self):
        constraintMatrixUpper = np.eye(self.ndim())
        constraintVectorUpper = self.boundsUpper
        constraintMatrixLower = -np.eye(self.ndim())
        constraintVectorLower = -self.boundsLower

        self.constraintMatrix = np.vstack((constraintMatrixUpper, constraintMatrixLower))
        self.constraintVector = np.hstack((constraintVectorUpper, constraintVectorLower))

class IndividualControl(Task):
    # task for controling each arm seperatly 
    def __init__(self, Dof):
        super(IndividualControl, self).__init__(Dof)
        self.constraintType = 0

    def compute(self, controlVelocity, jacobian):
        effectorVelocities = controlVelocity #controlTarget.getIndividualTargetVelocity(k_p=Parameters.k_p_i, k_o=Parameters.k_o_i) # k is the gain for vel = vel + k*error
        self.constraintMatrix = jacobian
        self.constraintVector = effectorVelocities


class RelativeControl(Task):
    # Task for contorlling the arms relative to each other 
    def __init__(self, Dof):
        super(RelativeControl, self).__init__(Dof)
        self.constraintType = 0
    
    def compute(self, controlVelocity, jacobian, transformer, yumiGripperPoseR, yumiGripperPoseL):
        #velocities, tfRightArm, tfLeftArm, absoluteOrientation = controlTarget.getRelativeTargetVelocity(k_p=Parameters.k_p_r, k_o=Parameters.k_o_r) # k is the gain for vel = vel + k*error
        avgQ = np.vstack([yumiGripperPoseR.getQuaternion(), yumiGripperPoseL.getQuaternion()])
        absoluteOrientation = utils.averageQuaternions(avgQ)
        tfMatrix = transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=absoluteOrientation)

        rotaionMatrix = np.linalg.pinv(tfMatrix[0:3,0:3])

        diffXYZ = tfRightArm - tfLeftArm
        skewMatrixDiff = np.array([[0,-diffXYZ[2],diffXYZ[1]],\
                                    [diffXYZ[2],0,-diffXYZ[0]],\
                                    [-diffXYZ[1],diffXYZ[0],0]])

        rotationSkew = 0.5*rotaionMatrix.dot(skewMatrixDiff)
        # linking matrix that maps both grippers velocity to the velocity difference in the absolute frame (average frame of the grippers)
        linkJ  = np.asarray(np.bmat([[rotaionMatrix, rotationSkew, -rotaionMatrix, rotationSkew],\
                            [np.zeros((3,3)), rotaionMatrix, np.zeros((3,3)), -rotaionMatrix]]))
        
        relativeJacobian = linkJ.dot(jacobian)
        self.constraintMatrix = relativeJacobian
        self.constraintVector = controlVelocity


class AbsoluteControl(Task):
    # Task for contorling the average of the grippers
    def __init__(self, Dof):
        super(AbsoluteControl, self).__init__(Dof)
        self.constraintType = 0
    
    def compute(self, controlVelocity, jacobian):
        #velocities = controlTarget.getAbsoluteTargetVelocity(k_p=Parameters.k_p_a, k_o=Parameters.k_o_a) # k is the gain for vel = vel + k*error
        # linking matrox that maps the grippers to the average of the grippers
        linkJ = np.hstack([0.5*np.eye(6), 0.5*np.eye(6)])
        absoluteJacobian = linkJ.dot(jacobian)
        self.constraintMatrix = absoluteJacobian
        self.constraintVector = controlVelocity


class ElbowProximityV2(Task):
    # Taks for keeping a proximity between the elbows of the robot 
    def __init__(self, Dof, minDistance, timestep):
        super(ElbowProximityV2, self).__init__(Dof, 1e3)
        
        self.constraintType = 1
        self.timestep = timestep
        self.minDistance = minDistance

    def compute(self, jacobianRightElbow, jacobianLeftElbow, yumiElbowPoseR, yumiElbowPoseL):
       
        translationRight = yumiElbowPoseR.getPosition()
        translationLeft = yumiElbowPoseL.getPosition()
        diff = translationRight[1] - translationLeft[1]
        d = np.linalg.norm(diff)
        jacobianNew = np.zeros((2, self.Dof))
        jacobianNew[0, 0:4] = jacobianRightElbow[1,0:4]
        jacobianNew[1, 7:11] = jacobianLeftElbow[1,0:4]
        LinkVelDiff = np.array([1, -1])

        self.constraintMatrix =  -self.timestep *10* diff * (LinkVelDiff.dot(jacobianNew)) / d
        self.constraintMatrix = np.expand_dims(self.constraintMatrix, axis=0)
        self.constraintVector = -np.array([(self.minDistance - d)])

class EndEffectorProximity(Task):
    # Task for keeping minimum proximity between the grippers 
    def __init__(self, Dof, minDistance, timestep):
        super(EndEffectorProximity, self).__init__(Dof, 1e3)
        self.constraintType = 1
        self.timestep = timestep
        self.minDistance = minDistance

    def compute(self, jacobian, yumiPoseR, yumiPoseL):
       
        translationRight = yumiPoseR.getPosition()
        translationLeft = yumiPoseL.getPosition()

        diff = translationRight[0:2] - translationLeft[0:2]
        d = np.linalg.norm(diff)
        jacobianNew = np.zeros((4, self.Dof))
        jacobianNew[0:2, 0:7] = jacobian[0:2,0:7]
        jacobianNew[2:4, 7:14] = jacobian[6:8,7:14]
        LinkVelDiff = np.array([[1,0, -1, 0],[0,1,0,-1]])

        self.constraintMatrix =  -self.timestep *10* diff.dot(LinkVelDiff.dot(jacobianNew)) / d
        self.constraintMatrix = np.expand_dims(self.constraintMatrix, axis=0)
        self.constraintVector = -np.array([(self.minDistance - d)])
 

class JointPositionPotential(Task):
    # Task for keeping a good joint configuration. 
    def __init__(self, Dof, defaultPose, timestep):
        super(JointPositionPotential, self).__init__(Dof, 2e2)
        self.timestep = timestep
        self.defaultPose = defaultPose
        self.constraintType = 0

    def compute(self, jointState):
        
        self.constraintMatrix = 100 * self.timestep * np.eye(self.ndim())
        vec = (self.defaultPose - jointState.jointPosition)/2
        # less strick on the last wrist joint. 
        vec[6] = vec[6]/2
        vec[13] = vec[13]/2

        self.constraintVector = vec

