#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from abb_rapid_sm_addin_msgs.srv import SetSGCommand
from abb_robot_msgs.srv import TriggerWithResultCode
from std_msgs.msg import Float64MultiArray

class JointState(object):
    """Class for storing the joint state"""
    def __init__(self,
                 jointPosition=np.array([1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0]),
                 jointVelocity=np.zeros(14)):
        """Initial joint position and velocity"""
        self.jointPosition = np.copy(jointPosition)  # only arms not gripper
        self.jointVelocity = np.copy(jointVelocity)  # only arms not gripper
    
    def GetJointVelocity(self):
        """"Returns the joint velocity, np.array([right, left])"""
        return np.copy(self.jointVelocity)
    
    def GetJointPosition(self):
        """"Returns the joint velocity, np.array([right, left])"""
        return np.copy(self.jointPosition)

    def UpdatePose(self, pose):
        """Update the joint positions , np.array([right, left])"""
        self.jointPosition = pose[0:14]

    def UpdateVelocity(self, velocity):
        """Update the joint velocities , np.array([right, left])"""
        self.jointVelocity = velocity[0:14]


class VelocityCommand(object):
    """Used for storing the velocity command for yumi"""
    def __init__(self):
        self.jointVelocity = np.zeros(14)

    def setVelocity(self, vel):
        """vel should be an np.array() with 14 elements, [right arm, left arm]"""
        self.jointVelocity = vel
    
    def getVelocity(self):
        """returns velocity numpy array [right arm, left arm]"""
        return np.copy(self.jointVelocity)

    def getVelocityPublish(self):
        """ returns python list [left arm, right arm]"""
        return np.hstack([self.jointVelocity[7:14], self.jointVelocity[0:7]]).tolist()


class TfBroadcastFrames(object):
    """class for adding new frames to the tf tree and used internally for control"""
    def __init__(self, gripperRight, gripperLeft, yumiToWorld, harnessBord):
        """:param gripperRight: class instance of FramePose describing the local transformation from yumi_link_7_r
        to yumi_grip_r
         :param gripperLeft: class instance of FramePose describing the local transformation from yumi_link_7_l
        to yumi_grip_l
        :param yumiToWorld: class instance of FramePose describing the local transformation from yumi_base_link
        to world
        :param harnessBord: class instance of FramePose describing the local transformation from yumi_base_link
        to harnessBord"""
        self.tfbrodcaster = tf.TransformBroadcaster()
        self.gripperRight = gripperRight
        self.gripperLeft = gripperLeft
        self.yumiToWorld = yumiToWorld
        self.harnessBord = harnessBord

    def updateGripperRight(self, gripperRight):
        """Update the frame
        :param gripperRight: class instance of FramePose describing the local transformation from yumi_link_7_r
        to yumi_grip_r"""
        self.gripperRight = gripperRight

    def updateGripperLeft(self, gripperLeft):
        """Update the frame
          :param gripperLeft: class instance of FramePose describing the local transformation from yumi_link_7_l
        to yumi_grip_l"""
        self.gripperLeft = gripperLeft

    def updateYumiToWorld(self, yumiToWorld):
        """Update the frame
          :param yumiToWorld: class instance of FramePose describing the local transformation from yumi_base_link
        to world"""
        self.yumiToWorld = yumiToWorld

    def updateHarnessBord(self, harnessBord):
        """Update the frame
          :param harnessBord: class instance of FramePose describing the local transformation from yumi_base_link
        to world"""
        self.harnessBord = harnessBord

    def getGripperRight(self):
        """Returns the frame"""
        return self.gripperRight

    def getGripperLeft(self):
        """Returns the frame"""
        return self.gripperLeft

    def getYumiToWorld(self):
        """Returns the frame"""
        return self.yumiToWorld
    
    def getHarnessBord(self):
        """Returns the frame"""
        return self.harnessBord

    def tfBroadcast(self):
        """Sends out to the tf tree"""
        self.tfbrodcaster.sendTransform(tuple(self.yumiToWorld.getPosition()),
                                        tuple(self.yumiToWorld.getQuaternion()),
                                        rospy.Time.now(), "world", "yumi_base_link")
        
        self.tfbrodcaster.sendTransform(tuple(self.harnessBord.getPosition()),
                                        tuple(self.harnessBord.getQuaternion()),
                                        rospy.Time.now(), "yumi_base_link", "harnessBord")

        self.tfbrodcaster.sendTransform(tuple(self.gripperRight.getPosition()),
                                        tuple(self.gripperRight.getQuaternion()),
                                        rospy.Time.now(), "yumi_grip_r", "yumi_link_7_r")

        self.tfbrodcaster.sendTransform(tuple(self.gripperLeft.getPosition()),
                                        tuple(self.gripperLeft.getQuaternion()),
                                        rospy.Time.now(), "yumi_grip_l", "yumi_link_7_l")


class FramePose(object):
    """Class for describing a frame or transformation"""
    def __init__(self, position=np.zeros(3), quaternion=np.zeros(4)):
        """:param position: np.array([x,y,z]) position [m]
        :param quaternion: np.array([x,y,z,w]) orientation [unit quaternion]"""
        self.position = np.copy(position)
        self.quaternion = np.copy(quaternion)
            
        self.tempPosition = np.zeros(3)
        self.tempQuaternion = np.zeros(4)

    def getQuaternion(self):
        """returns the quaternion"""
        return np.copy(self.quaternion)

    def getPosition(self):
        """returns the position"""
        return np.copy(self.position[0:3])

    def setPosition(self, position):
        """updates the position
        :param position: np.array([x,y,z]) [m]
        """
        self.position = position 

    def setQuaternion(self, quaternion):
        """updates the orientation
       :param quaternion: np.array([x,y,z,w]) orientation [unit quaternion]"""
        self.quaternion = quaternion

    def update_(self, pose, transformer=None, gripperLocalTransfrom=None):
        """Updates frame frame pose msg
        :param pose: pose msg (ros)
        :param transformer: tf.transformer (only needed if gripperLocalTransfrom)
        :param gripperLocalTransfrom: (FramePose instance) local transformation of be added to pose"""
        if transformer == None or gripperLocalTransfrom == None:
            self.position[0] = pose.position.x
            self.position[1] = pose.position.y
            self.position[2] = pose.position.z
            self.quaternion[0] = pose.orientation.x
            self.quaternion[1] = pose.orientation.y
            self.quaternion[2] = pose.orientation.z
            self.quaternion[3] = pose.orientation.w
        else:
            self.tempPosition[0] = pose.position.x
            self.tempPosition[1] = pose.position.y
            self.tempPosition[2] = pose.position.z

            self.tempQuaternion[0] = pose.orientation.x
            self.tempQuaternion[1] = pose.orientation.y
            self.tempQuaternion[2] = pose.orientation.z
            self.tempQuaternion[3] = pose.orientation.w
            
            self.quaternion = tf.transformations.quaternion_multiply(self.tempQuaternion,
                                                                     gripperLocalTransfrom.getQuaternion())
            tfMatrix = transformer.fromTranslationRotation(translation=self.tempPosition, rotation=self.tempQuaternion)
            self.position = tfMatrix.dot(np.hstack([gripperLocalTransfrom.getPosition(), 1]))


def CalcJacobianCombined(data, gripperLocalTransform, transformer, yumiGripPoseR, yumiGripPoseL):
    """Combines the Jacobians for both arms and extends them to include a new frame as grip point
    :param data: Jacobian msg
    :param gripperLocalTransform: TfBroadcastFrames instance
    :param transformer: tf.transformer
    :param yumiGripPoseR: FramePose of right gripper
    :param yumiGripPoseL: FramePose of left gripper
    :returns combined and extended Jacobian"""
    dataNP = np.asarray(data.data)

    jacobianRightArm = dataNP[0::2].reshape((6, 7))
    jacobianLeftArm = dataNP[1::2].reshape((6, 7))
    
    # change end-effector frame
    rotationRightArm = yumiGripPoseR.getQuaternion()
    rotationLeftArm = yumiGripPoseL.getQuaternion()

    jacobianRightArm = changeFrameJacobian(jacobianRightArm, gripperLocalTransform.getGripperRight(), rotationRightArm,
                                           transformer)
    jacobianLeftArm = changeFrameJacobian(jacobianLeftArm, gripperLocalTransform.getGripperLeft(), rotationLeftArm,
                                          transformer)
    
    return np.asarray(np.bmat([[jacobianRightArm, np.zeros((6, 7))], [np.zeros((6, 7)), jacobianLeftArm]]))


def changeFrameJacobian(jacobian, gripperLocal, rotation, transformer):
    """Extends the Jacobian with a new frame
    :param jacobian: np.array() shape(6,7)
    :param gripperLocal, FramePose of local transformation from end-effector
    :param rotation: np.array() shape(4) unit quaternion describing orientation of end-effector
    :param transformer: tf.transformer"""
    # change end-effector for jacobian, velocity now depends on angular vel and tranlational vel
    transformationMatrix = transformer.fromTranslationRotation(translation=np.array([0, 0, 0]), rotation=rotation)
    velocityXYZ = transformationMatrix[0:3, 0:3].dot(gripperLocal.getPosition().reshape((3, 1)))
    eye3 = np.eye(3)
    zeros3 = np.zeros((3, 3))
    linkRotation = np.array([[0, velocityXYZ[2, 0], -velocityXYZ[1, 0]], [-velocityXYZ[2, 0], 0, velocityXYZ[0, 0]],
                             [velocityXYZ[1, 0], -velocityXYZ[0, 0], 0]])
    linkingMatrix = np.asarray(np.bmat([[eye3, linkRotation], [zeros3, eye3]]))
    return linkingMatrix.dot(jacobian)


class GripperControl(object):
    """ class for controlling the grippers on YuMi, the grippers are controlled in [mm] and uses ros service"""
    def __init__(self):
        #rosservice, for control over grippers
        self.SetSGCommand = rospy.ServiceProxy('/yumi/rws/sm_addin/set_sg_command', SetSGCommand)
        self.RunSGRoutine = rospy.ServiceProxy('/yumi/rws/sm_addin/run_sg_routine', TriggerWithResultCode)
        self.lastGripperLeft = 0
        self.lastGripperRight = 0
        
        # For when simulation is running
        self.pubGripperSim = rospy.Publisher('/sim/grippers', Float64MultiArray, queue_size=1) 

    def setGripperPosition(self, gripperRight=None, gripperLeft=None):
        """Set new gripping position
        :param gripperRight: float [mm]
        :param gripperLeft: float [mm]
        """
        tol = 1e-5
        try:
            # stacks/set the commands for the grippers 
            # to not sent same command twice. As the grippers momentarily regripps if the same command is sent twice.
            # for left gripper
            if gripperLeft is not None:
                if abs(self.lastGripperLeft - gripperLeft) >= tol:
                    if gripperLeft <= 0.1: # if gripper set close to zero then grip in 
                        self.SetSGCommand(task="T_ROB_L", command=6)
                    else: # otherwise move to position 
                        self.SetSGCommand(task="T_ROB_L", command=5, target_position=gripperLeft)
                    self.lastGripperLeft = gripperLeft
                
            # for right gripper
            if gripperRight is not None:
                if abs(self.lastGripperRight - gripperRight) >= tol:
                    if gripperRight <= 0.1:
                        self.SetSGCommand(task="T_ROB_R", command=6)
                    else:
                        self.SetSGCommand(task="T_ROB_R", command=5, target_position=gripperRight)

                    self.lastGripperRight = gripperRight
                    
            # sends of the commands to the robot
            self.RunSGRoutine()

        except:
            print('smart gripper error')


            

class TrajectoryPoint(object):
    """Class for storing trajectory points"""
    def __init__(self,
            positionLeft=np.array([0.4 ,0.2, 0.2]),
            positionRight=np.array([0.4 ,-0.2, 0.2]),
            orientationLeft=np.array([1,0,0,0]),
            orientationRight=np.array([1,0,0,0]),
            gripperLeft=0,
            gripperRight=0,
            pointTime=2.0):
        self.positionLeft = positionLeft
        self.positionRight = positionRight
        self.orientationLeft = orientationLeft
        self.orientationRight = orientationRight
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.pointTime = pointTime


# used to calculate error
def PositionError(currentPositionXYZ, targetPositionXYZ, maxVelocity):
    """Calculates a clipped position error
    :param currentPositionXYZ: np.array shape(3) [m]
    :param targetPositionXYZ: np.array shape(3) [m]
    :param maxVelocity: max m/s"""
    positionDiff = (targetPositionXYZ - currentPositionXYZ) 
    norm = np.linalg.norm(positionDiff)
    positionDiffNormalized = normalize(positionDiff)        
    return positionDiffNormalized*min([maxVelocity, norm])


# used to calculate error
def RotationError(currentQ, targetQ, maxRotVel):
    """Calculates a clipped angular error
    :param currentQ: quaternion np.array() shape(4)
    :param targetQ: quaternion np.array() shape(4)
    :param maxRotVel: max rad/s"""
    if currentQ.dot(targetQ) < 0:
        currentQ = -currentQ

    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],
                           [targetQ[2], 0, -targetQ[0]],
                           [-targetQ[1], targetQ[0], 0]])

    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3])
    norm = np.linalg.norm(errorOrientation)
    errorOrientationNormalized = normalize(errorOrientation)
    return errorOrientationNormalized*min([maxRotVel, norm])


def normalize(v):
    """Calculates the normalized vector
    :param v: np.array()"""
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm


# taken (Modified) from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (x,y,z,w), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    """Calcualtes the average quaternion between N quaternions
    :param Q: np.array() shape(N, 4)"""
    if Q[0].dot(Q[1]) < 0:
        Q[0] = -Q[0]
        
    # from (x,y,z,w) to (w,x,y,z)
    Q = np.roll(Q, 1, axis=1)
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4, 4))
    for i in range(0, M):
        q = Q[i, :]
        # multiply q with its transposed version q' and add A
        A = np.outer(q, q) + A
    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    avgQ = np.real(eigenVectors[:, 0])
    # from (w,x,y,z) to (x,y,z,w) 
    avgQ = np.roll(avgQ, -1)
    return avgQ


def calcPosVel(qi, dqi, qf, dqf, tf, t): 
    """outputs target position and velocity, cubic interpolation between points
        :param qi: initial pos (np.array() shape(N))
        :param dqi: initial vel (np.array() shape(N))
        :param qf: final pos (np.array() shape(N))
        :param dqf: final vel (np.array() shape(N))
        :param tf: total time between points [s]
        :param t: current time 0 <= t <= tf [s] """
    num = np.shape(qi)[0]
    q = np.zeros(num)
    dq = np.zeros(num)
    for k in range(num):
        a0 = qi[k]
        a1 = dqi[k]
        a2 = 3 * (qf[k] - (dqf[k]*tf)/3 - a1*tf*(2/3) - a0)/(tf*tf)
        a3 = (dqf[k] - (2*a2*tf + a1))/(3*tf*tf)
        q[k] = a3*t**3 + a2*t**2 + a1*t + a0
        dq[k] = 3*a3*t**2 + 2*a2*t + a1
    return q, dq


