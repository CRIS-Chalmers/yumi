#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from abb_rapid_sm_addin_msgs.srv import SetSGCommand
from abb_robot_msgs.srv import TriggerWithResultCode
from std_msgs.msg import Float64MultiArray, Float64, Int64

class JointState(object):
    def __init__(self,\
            jointPosition=np.array([1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0]),\
            jointVelocity=np.zeros(14)):
        self.jointPosition = jointPosition # only arm not gripper
        self.jointVelocity = jointVelocity # only arm not gripper

    
    def GetJointVelocity(self):
        return np.hstack([self.jointVelocity])
    
    def GetJointPosition(self):
        return np.hstack([self.jointPosition])

    def UpdatePose(self, pose):
        self.jointPosition = pose[0:14]

    def UpdateVelocity(self, vel):
        self.jointVelocity = vel[0:14]


class TrajectoryPoint(object):
    def __init__(self,\
            positionLeft=np.array([0.4 ,0.2, 0.2]),\
            positionRight=np.array([0.4 ,-0.2, 0.2]),\
            orientationLeft=np.array([1,0,0,0]),\
            orientationRight=np.array([1,0,0,0]),\
            gripperLeft=0,\
            gripperRight=0,\
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
    positionDiff = (targetPositionXYZ - currentPositionXYZ) 
    norm = np.linalg.norm(positionDiff)
    positionDiffNormalized = normalize(positionDiff)        
    return positionDiffNormalized*min([maxVelocity, norm])

# used to calculate error
def RotationError(currentQ, targetQ, maxRotVel):

    if currentQ.dot(targetQ) < 0:
        currentQ = -currentQ

    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],\
                            [targetQ[2],0,-targetQ[0]],\
                                [-targetQ[1],targetQ[0],0]])

    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3] )
    norm = np.linalg.norm(errorOrientation)

    errorOrientationNormalized = normalize(errorOrientation)     
    #errorOrientationNormalized*min([maxRotVel, norm])   
    return errorOrientationNormalized*min([maxRotVel, norm])


def CalcJacobianCombined(data, gripperLengthRight, gripperLengthLeft, transformer, yumiGrippPoseR, yumiGrippPoseL):
    jacobianRightArm = np.zeros((6,7))
    jacobianLeftArm = np.zeros((6,7))

    dataNP = np.asarray(data.data)

    jacobianRightArm = dataNP[0::2].reshape((6,7))
    jacobianLeftArm = dataNP[1::2].reshape((6,7))
    
    # change endeffector frame 
    translationRightArm = yumiGrippPoseR.getPosition()
    translationLeftArm = yumiGrippPoseL.getPosition()
    rotationRightArm = yumiGrippPoseR.getQuaternion()
    rotationLeftArm = yumiGrippPoseL.getQuaternion()

    jacobianRightArm = changeFrameJacobian(jacobianRightArm, gripperLengthRight, rotationRightArm, transformer)
    jacobianLeftArm = changeFrameJacobian(jacobianLeftArm, gripperLengthLeft, rotationLeftArm, transformer)
    
    return np.asarray(np.bmat([[jacobianRightArm,np.zeros((6,7))],[np.zeros((6,7)),jacobianLeftArm]]))


def changeFrameJacobian(jacobian, gripperLenght, rotation, transformer):
    # change end effector for jacobian 
    transformationMatrix = transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=rotation)
    gripperLenght = np.asarray(gripperLenght)
    velocityXYZ = transformationMatrix[0:3,0:3].dot(gripperLenght.reshape((3,1)))
    eye3 = np.eye(3)
    zeros3 = np.zeros((3,3))
    linkRotation = np.array([[0, velocityXYZ[2,0], -velocityXYZ[1,0]],[-velocityXYZ[2,0],0,velocityXYZ[0,0]],\
        [velocityXYZ[1,0],-velocityXYZ[0,0],0]])
    linkingMatrix = np.asarray(np.bmat([[eye3,linkRotation],[zeros3,eye3]]))
    return linkingMatrix.dot(jacobian)


def normalize(v):
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
    if Q[0].dot(Q[1]) < 0:
        Q[0] = -Q[0]
        
    # from (x,y,z,w) to (w,x,y,z)

    Q = np.roll(Q,1,axis=1)
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4,4))
    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A
    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    avgQ = np.real(eigenVectors[:,0])
    # from (w,x,y,z) to (x,y,z,w) 
    avgQ = np.roll(avgQ,-1)

    return avgQ

class FramePose(object):
    def __init__(self):
        self.tempPosition = np.zeros(3)
        self.position = np.zeros(4)
        self.quaternion = np.zeros(4)

    def getQuaternion(self):
        return self.quaternion

    def getPosition(self):
        return self.position[0:3]

    def update(self, pose, transfromer, gripperLength):
        self.tempPosition[0] = pose.position.x
        self.tempPosition[1] = pose.position.y
        self.tempPosition[2] = pose.position.z

        self.quaternion[0] = pose.orientation.x
        self.quaternion[1] = pose.orientation.y
        self.quaternion[2] = pose.orientation.z
        self.quaternion[3] = pose.orientation.w

        tfMatrix = transfromer.fromTranslationRotation(translation=self.tempPosition, rotation=self.quaternion)
        self.position = tfMatrix.dot(np.hstack([gripperLength, 1]))


class GripperControl(object):
    # class for controlling the grippers on YuMi, the grippers are controlled in [mm]
    def __init__(self):
         #rosservice, for control over grippers
        self.SetSGCommand = rospy.ServiceProxy('/yumi/rws/sm_addin/set_sg_command', SetSGCommand)
        self.RunSGRoutine = rospy.ServiceProxy('/yumi/rws/sm_addin/run_sg_routine', TriggerWithResultCode)
        self.lastGripperLeft = 0
        self.lastGripperRight = 0
        
        # For when simulation is running
        self.pubGripperSim = rospy.Publisher('/sim/grippers', Float64MultiArray, queue_size=1) 

    def setGripperPosition(self, gripperRight, gripperLeft): # input in [mm]
        tol = 1e-5
        try:
            # stacks/set the commands for the grippers 
            # to not sent same command twice. As the grippers momentarly regripps if the same command is sent twice. 

            # for left gripper
            if abs(self.lastGripperLeft - gripperLeft) >= tol:
                if gripperLeft <= 0.1: # if gripper set close to zero then grip in 
                    self.SetSGCommand(task="T_ROB_L", command=6) 
                else: # otherwise move to position 
                    self.SetSGCommand(task="T_ROB_L", command=5, target_position=gripperLeft)
                self.lastGripperLeft = gripperLeft
            
            # for right gripper
            if abs(self.lastGripperRight - gripperRight) >= tol:
                if gripperRight <= 0.1:
                    self.SetSGCommand(task="T_ROB_R", command=6)
                else:
                    self.SetSGCommand(task="T_ROB_R", command=5, target_position=gripperRight)

                self.lastGripperRight = gripperRight
            
            # sends of the commandes to the robot
            self.RunSGRoutine()

        except:
            # The rviz simumlation is running then the ros service will fail, normal ROS communication is used
            print('smart gripper error or running simulation')
            msg = Float64MultiArray()
            msg.data = [gripperRight, gripperLeft]
            self.pubGripperSim.publish(msg)

