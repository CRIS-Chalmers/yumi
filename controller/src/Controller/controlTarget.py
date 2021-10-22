#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from trajectory import Trajectory
from parameters import Paramters
import utils

class ControlTarget(object): # generates target velocity in task space
    def __init__(self, deltaTime):
        self.mode = 'individual' # can be either 'individual' or 'coordinated'
        self.trajectory = Trajectory(deltaTime) # instance of trajectory class
        self.targetVelocities = np.zeros(12) # in [m/s] and [rad/s]
        self.error = np.zeros(12) # in [m] and [rad]
        self.gripperLeft = 0 # Gripper poistion in [mm]
        self.gripperRight = 0

        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.trajectorySegment = 0 # keeps track of which trajectory segment currently executed

    def getIndividualTargetVelocity(self, k_p, k_o):
        # calculates the target velocities for individual control 

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        self.error[0:3] = utils.PositionError(self.translationRightArm,\
                                            self.targetPosition[0:3], 0.1)
        self.error[6:9] = utils.PositionError(self.translationLeftArm,\
                                            self.targetPosition[3:6], 0.1)
        self.error[3:6]= utils.RotationError(self.rotationRightArm, \
                                            self.targetOrientation[0:4], 0.2)
        self.error[9:12]= utils.RotationError(self.rotationLeftArm, \
                                            self.targetOrientation[4:8], 0.2)

        K = np.array([k_p,k_p,k_p, k_o,k_o,k_o, k_p,k_p,k_p, k_o,k_o,k_o])
        
        self.targetVelocities = self.desierdVelocity + K*self.error

        return self.targetVelocities

    def getAbsoluteTargetVelocity(self, k_p, k_o):
        # calculates the target velocities for absolute motion, 
        # controlling the average of the grippers 

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        self.error[0:3] =  utils.PositionError(self.absolutePosition,\
                                            self.targetPosition[0:3], 0.1)
        self.error[3:6] =  utils.RotationError(self.absoluteOrientation, \
                                            self.targetOrientation[0:4], 0.2)

        K = np.array([k_p,k_p,k_p, k_o,k_o,k_o])

        self.targetVelocities[0:6] = self.desierdVelocity[0:6] + K*self.error[0:6]

        return self.targetVelocities[0:6]

    def getRelativeTargetVelocity(self,  k_p, k_o):
        # calculates the target velocities for relative motion, 
        # controlling the differnece of the grippers 
        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)
        
        self.error[6:9] = utils.PositionError(self.realativPosition ,\
                                            self.targetPosition[3:6], 0.1)

        self.error[9:12]= utils.RotationError(self.rotationRelative, \
                                            self.targetOrientation[4:8], 0.2)

        K = np.array([k_p,k_p,k_p, k_o,k_o,k_o])

        self.targetVelocities[6:12] = self.desierdVelocity[6:12]  + K*self.error[6:12]

        return self.targetVelocities[6:12]

    def checkNewTrajectorySegment(self):
        # returns True is a new segment has been entered, 
        # can only be called once for each segment, otherwise returns False
        # Used to check if new gripper commands should be sent.  
        if self.trajectorySegment != self.trajectory.index:
            self.trajectorySegment = self.trajectory.index
            return True
        else:
            return False

    def updateTarget(self):
        # updates the desierd velocities and target position from the trajectory
        if len(self.trajectory.trajectory) < 2: 
            return
        self.targetPosition, self.targetOrientation, self.desierdVelocity, self.gripperLeft, self.gripperRight = self.trajectory.getTarget()

    def updatePose(self, yumiGrippPoseR, yumiGrippPoseL):
        # updates the pose and calculates the pose for relative and absolute motion as well
        self.translationRightArm = yumiGrippPoseR.getPosition()
        self.translationLeftArm = yumiGrippPoseL.getPosition()
        self.rotationRightArm = yumiGrippPoseR.getQuaternion()
        self.rotationLeftArm = yumiGrippPoseL.getQuaternion()

        # absolute pose, avg of the grippers
        tfMatrixRight = self.transformer.fromTranslationRotation(translation=self.translationRightArm, rotation=self.rotationRightArm)
        tfMatrixLeft = self.transformer.fromTranslationRotation(translation=self.translationLeftArm, rotation=self.rotationLeftArm)

        avgQ = np.vstack([self.rotationRightArm, self.rotationLeftArm])
        self.absoluteOrientation = utils.averageQuaternions(avgQ)  
        self.absolutePosition = 0.5*(self.translationRightArm + self.translationLeftArm)

        # relative pose, diference of the grippers in absolute frame 
        transformationAbsolute = self.transformer.fromTranslationRotation(translation=self.absolutePosition, rotation=self.absoluteOrientation)
        transformationAbsoluteInv = np.linalg.pinv(transformationAbsolute)

        transformationRightFromAbs = transformationAbsoluteInv.dot(tfMatrixRight)
        transformationLeftFromAbs = transformationAbsoluteInv.dot(tfMatrixLeft)
        quatRightAbs = tf.transformations.quaternion_from_matrix(transformationRightFromAbs)
        posRightAbs = tf.transformations.translation_from_matrix(transformationRightFromAbs)
        quatLeftAbs = tf.transformations.quaternion_from_matrix(transformationLeftFromAbs)
        posLeftAbs = tf.transformations.translation_from_matrix(transformationLeftFromAbs)

        self.rotationRelative = tf.transformations.quaternion_multiply(quatRightAbs, tf.transformations.quaternion_conjugate(quatLeftAbs))
        self.realativPosition = posRightAbs - posLeftAbs

    def checkTrajectoryDevation(self):
        devation = np.max(np.abs(self.error) - Paramters.maxTrajectoryDeviaiton)
        
        # deacitvates check if gripperColllsionActive is active
        if self.mode == 'indidivual' and Paramters.gripperColllsionActive == 1:
            return True

        if devation > 0:
            return False
        return True

