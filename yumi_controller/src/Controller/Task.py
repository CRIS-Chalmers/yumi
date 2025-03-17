

import numpy as np
import os
import sys
import utils
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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

        elif self.constraintType == 1:
            G = np.hstack((self.constraintMatrix, -np.eye(m)))
            h = self.constraintVector
            A = np.zeros((1, G.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            b = np.zeros((1, ))                 # Does not do anything otherwise.

        elif self.constraintType == -1: # think this might have the wrong sign on the relaxation variable
            G = np.hstack((self.constraintMatrix, np.eye(m)))
            h = self.constraintVector
            A = np.zeros((1, G.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            b = np.zeros((1, ))                 # Does not do anything otherwise.

        return A, b, G, h
    


class JointPositionBoundsTaskV2(Task):
    """Task for keeping joint positions from saturating"""
    def __init__(self, Dof, boundsUpper, boundsLower, timestep):
        """:param Dof: degrees of freedom, 14 for YuMi
        :param boundsUpper: np.array() shape(14)
        :param boundsLower: np.array() shape(14)
        :param timestep: dt"""
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

        self.constraintMatrix = np.vstack((constraintMatrixUpper, constraintMatrixLower))
        self.constraintVector = np.hstack((constraintVectorUpper, constraintVectorLower))


class JointVelocityBoundsTaskV2(Task):
    """Task for keeping joint velocity within limits"""
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
    """ task for controlling each arm separately """
    def __init__(self, Dof):
        super(IndividualControl, self).__init__(Dof)
        self.constraintType = 0

    def compute(self, controlVelocity, jacobian):
        """Updates Jacobian and the target velocities
        :param controlVelocity: np.array([rightT, rightR, leftT, leftR]) shape(12)
        :param jacobian: np.array(), shape(12,14)"""
        self.constraintMatrix = jacobian
        self.constraintVector = controlVelocity


class RelativeControl(Task):
    """Task for contorlling the arms relative to each other"""
    def __init__(self, Dof):
        super(RelativeControl, self).__init__(Dof)
        self.constraintType = 0
    
    def compute(self, controlVelocity, jacobian, transformer, yumiGripperPoseR, yumiGripperPoseL):
        """ sets up the constraints for relative control.
            :param controlVelocity: np.array([relTrans, RelRot]) relative velocities between the grippers
            :param jacobian: np.array(), shape(12,14)
            :param transformer: tf.transformer
            :param yumiGripperPoseR: class (FramePose) describing the pose of the gripper
            :param yumiGripperPoseL: class (FramePose) describing the pose of the gripper """

        avgQ = np.vstack([yumiGripperPoseR.getQuaternion(), yumiGripperPoseL.getQuaternion()])
        absoluteOrientation = utils.averageQuaternions(avgQ)
        tfMatrix = transformer.fromTranslationRotation(translation=np.array([0, 0, 0]), rotation=absoluteOrientation)

        rotaionMatrix = np.linalg.pinv(tfMatrix[0:3, 0:3])

        diffXYZ = yumiGripperPoseR.getPosition() - yumiGripperPoseL.getPosition()
        skewMatrixDiff = np.array([[0, -diffXYZ[2], diffXYZ[1]],
                                   [diffXYZ[2], 0, -diffXYZ[0]],
                                   [-diffXYZ[1], diffXYZ[0], 0]])

        rotationSkew = 0.5*rotaionMatrix.dot(skewMatrixDiff)
        # linking matrix that maps both grippers velocity to the velocity difference in the absolute frame
        # (average frame of the grippers)
        linkJ = np.asarray(np.bmat([[rotaionMatrix, rotationSkew, -rotaionMatrix, rotationSkew],
                                    [np.zeros((3, 3)), rotaionMatrix, np.zeros((3, 3)), -rotaionMatrix]]))
        
        relativeJacobian = linkJ.dot(jacobian)
        self.constraintMatrix = relativeJacobian
        self.constraintVector = controlVelocity


class AbsoluteControl(Task):
    """ Task for controlling the average of the grippers """
    def __init__(self, Dof):
        super(AbsoluteControl, self).__init__(Dof)
        self.constraintType = 0
    
    def compute(self, controlVelocity, jacobian):
        """ sets up the constraints for relative control.
                :param controlVelocity: np.array([relTrans, RelRot]) velocity of the average of the grippers
                :param jacobian: np.array(), shape(12,14)"""
        # linking matrix that maps the grippers to the average of the grippers
        linkJ = np.hstack([0.5*np.eye(6), 0.5*np.eye(6)])
        absoluteJacobian = linkJ.dot(jacobian)
        self.constraintMatrix = absoluteJacobian
        self.constraintVector = controlVelocity


class ElbowProximityV2(Task):
    """Task for keeping a proximity between the elbows of the robot"""
    def __init__(self, Dof, minDistance, timestep):
        super(ElbowProximityV2, self).__init__(Dof, 1e3)
        
        self.constraintType = 1
        self.timestep = timestep
        self.minDistance = minDistance

    def compute(self, jacobianRightElbow, jacobianLeftElbow, yumiElbowPoseR, yumiElbowPoseL):
        """ sets up the constraint for elbow proximity
                :param jacobianRightElbow: Jacobian for the right elbow
                :param jacobianLeftElbow: Jacobian for the left elbow
                :param yumiElbowPoseR: class (FramePose) describing the pose of the elbow
                :param yumiElbowPoseL: class (FramePose) describing the pose of the elbow"""

        translationRight = yumiElbowPoseR.getPosition()
        translationLeft = yumiElbowPoseL.getPosition()
        diff = translationRight[1] - translationLeft[1]
        d = np.linalg.norm(diff)
        jacobianNew = np.zeros((2, self.Dof))
        jacobianNew[0, 0:4] = jacobianRightElbow[1, 0:4]
        jacobianNew[1, 7:11] = jacobianLeftElbow[1, 0:4]
        LinkVelDiff = np.array([1, -1])

        self.constraintMatrix = -self.timestep * 10 * diff * (LinkVelDiff.dot(jacobianNew)) / d
        self.constraintMatrix = np.expand_dims(self.constraintMatrix, axis=0)
        self.constraintVector = -np.array([(self.minDistance - d)])


class EndEffectorProximity(Task):
    """ Task for keeping minimum proximity between the grippers """
    def __init__(self, Dof, minDistance, timestep):
        super(EndEffectorProximity, self).__init__(Dof, 1e3)
        self.constraintType = 1
        self.timestep = timestep
        self.minDistance = minDistance

    def compute(self, jacobian, yumiGripperPoseR, yumiGripperPoseL):
        """ sets up the constraints collision avoidance, i.e. the grippers will deviate from control command in order
        to not collide.
                :param jacobian: np.array(), shape(12,14)
                :param yumiGripperPoseR: class (FramePose) describing the pose of the gripper
                :param yumiGripperPoseL: class (FramePose) describing the pose of the gripper """
        translationRight = yumiGripperPoseR.getPosition()
        translationLeft = yumiGripperPoseL.getPosition()

        diff = translationRight[0:2] - translationLeft[0:2]
        d = np.linalg.norm(diff)
        jacobianNew = np.zeros((4, self.Dof))
        jacobianNew[0:2, 0:7] = jacobian[0:2, 0:7]
        jacobianNew[2:4, 7:14] = jacobian[6:8, 7:14]
        LinkVelDiff = np.array([[1, 0, -1, 0], [0, 1, 0, -1]])

        self.constraintMatrix = -self.timestep * 10 * diff.dot(LinkVelDiff.dot(jacobianNew)) / d
        self.constraintMatrix = np.expand_dims(self.constraintMatrix, axis=0)
        self.constraintVector = -np.array([(self.minDistance - d)])
 

class JointPositionPotential(Task):
    """ Task for keeping a good joint configuration. """
    def __init__(self, Dof, defaultPose, timestep):
        super(JointPositionPotential, self).__init__(Dof, 2e2)
        self.timestep = timestep
        self.defaultPose = defaultPose
        self.constraintType = 0

    def compute(self, jointState):
        """ Sets up constraints for joint potential,
            :param jointState: class (JointState)"""
        self.constraintMatrix = 100 * self.timestep * np.eye(self.ndim())
        vec = (self.defaultPose - jointState.jointPosition)/2
        # less strict on the last wrist joint.
        vec[6] = vec[6]/2
        vec[13] = vec[13]/2

        self.constraintVector = vec

