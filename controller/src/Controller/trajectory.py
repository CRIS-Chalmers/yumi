#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import utils

class Trajectory(object):
    """ Generates a trajectory from trajectory parameters, the process is identical for individual and coordinated manipulation.
     The variable names follows individual motion with left and right. This means when coordinate manipulation is used, right
     is absolute motion and left becomes relative motion. """
    def __init__(self, deltaTime):
        self.trajectory = []  # stores trajectory parameters
        self.trajectoryTime = 0  # keeps track on time, resets for every trajectory segment
        self.deltaTime = deltaTime
        self.index = 1  # current target trajectory parameter, 0 is the current position.
        self.numberOfTrajectoryParameters = 0
        self.positionVelocitiesLeft = []  # stores velocity transitions between trajectory parameters
        self.positionVelocitiesRight = []
        self.rotationMatrixLeft = []  # stores rotation matrices to save some computation
        self.rotationMatrixRight = []
        self.desiredVelocity = np.zeros(12)  # desired velocity from trajectory
        self.targetPosition = np.zeros(6)  # corresponding target position and orientation
        self.targetOrientation = np.zeros(8)
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

    def updateTrajectory(self, trajcetoryParameters, velLeftInit, velRightInit):
        """ updates the trajectory when a new trajectory as been received
            :param trajcetoryParameters: list of trajectory points,
            :param velLeftInit: np.array() shape(3) initial position velocity
            :param velRightInit: np.array() shape(3) initial position velocity
        """

        self.positionVelocitiesLeft = [velLeftInit]  # current velocity of the robot
        self.positionVelocitiesRight = [velRightInit]
        self.rotationMatrixLeft = []  # reset rotation matrices
        self.rotationMatrixRight = []
        # reset parameters
        self.index = 1 
        self.trajectoryTime = 0
        self.trajectory = trajcetoryParameters
        self.numberOfTrajectoryParameters = len(self.trajectory)

        # calculate list of rotation matrices for each trajectory parameters
        for i in range(0, self.numberOfTrajectoryParameters):
            tfMatrixRight = self.transformer.fromTranslationRotation(translation=np.zeros(3),
                                                                     rotation=trajcetoryParameters[i].orientationRight)
            tfMatrixLeft = self.transformer.fromTranslationRotation(translation=np.zeros(3),
                                                                    rotation=trajcetoryParameters[i].orientationLeft)
            self.rotationMatrixLeft.append(tfMatrixLeft[0:3, 0:3])
            self.rotationMatrixRight.append(tfMatrixRight[0:3, 0:3])

        # calculate the transition velocities between the trajectory parameters.
        for i in range(1, self.numberOfTrajectoryParameters-1):
            vel = self.calcPointVel(trajcetoryParameters[i-1].positionRight, trajcetoryParameters[i].positionRight,
                                    trajcetoryParameters[i+1].positionRight, trajcetoryParameters[i].pointTime,
                                    trajcetoryParameters[i+1].pointTime)
            self.positionVelocitiesRight.append(vel)
            vel = self.calcPointVel(trajcetoryParameters[i-1].positionLeft, trajcetoryParameters[i].positionLeft,
                                    trajcetoryParameters[i+1].positionLeft, trajcetoryParameters[i].pointTime,
                                    trajcetoryParameters[i+1].pointTime)
            self.positionVelocitiesLeft.append(vel)

        # last point always has velocity 0.
        self.positionVelocitiesRight.append(np.zeros(3))
        self.positionVelocitiesLeft.append(np.zeros(3))


    def getTarget(self):
        """ calculates the desired velocities and target pose. """

        # updates the current target trajectory parameters or which is the current segment on the trajectory
        if self.trajectory[self.index].pointTime < self.trajectoryTime:
            if self.index < self.numberOfTrajectoryParameters - 1:
                self.trajectoryTime = 0
                self.index += 1 
            else:  # for last point
                self.trajectoryTime = self.trajectory[self.index].pointTime
                self.index = self.numberOfTrajectoryParameters - 1

        # calculates the target position and desired velocity for a time point
        q, dq = utils.calcPosVel(qi=self.trajectory[self.index-1].positionRight,
                                 dqi=self.positionVelocitiesRight[self.index-1],
                                 qf=self.trajectory[self.index].positionRight,
                                 dqf=self.positionVelocitiesRight[self.index],
                                 tf=self.trajectory[self.index].pointTime,
                                 t=self.trajectoryTime)
        self.targetPosition[0:3] = q
        self.desiredVelocity[0:3] = dq

        q, dq = utils.calcPosVel(qi=self.trajectory[self.index-1].positionLeft,
                                 dqi=self.positionVelocitiesLeft[self.index-1],
                                 qf=self.trajectory[self.index].positionLeft,
                                 dqf=self.positionVelocitiesLeft[self.index],
                                 tf=self.trajectory[self.index].pointTime,
                                 t=self.trajectoryTime)
        self.targetPosition[3:6] = q
        self.desiredVelocity[6:9] = dq

        # calculates the target orientation and desierd angular velocity for a time point
        quat, we = self.calcOrientation(Ri=self.rotationMatrixRight[self.index-1],
                                        Rf=self.rotationMatrixRight[self.index])
        self.targetOrientation[0:4] = quat
        self.desiredVelocity[3:6] = we

        quat, we = self.calcOrientation(Ri=self.rotationMatrixLeft[self.index-1],
                                        Rf=self.rotationMatrixLeft[self.index])

        self.targetOrientation[4:8] = quat
        self.desiredVelocity[9:12] = we

        # update time 
        self.trajectoryTime += self.deltaTime

        return self.targetPosition, self.targetOrientation, self.desiredVelocity,\
               self.trajectory[self.index].gripperLeft, self.trajectory[self.index].gripperRight

    def calcOrientation(self, Ri, Rf): 
        """ outputs target orientation and angular velocity
         cubic interpolation between points, but always zero transition velocity
            :param Ri: rotation matrix describing the start of a segment
            :param Rf: rotation matrix describing the final orientation of a segment
            :Returns desired orientation and angular velocity"""
        R_i_f = np.transpose(Ri).dot(Rf)
        inCos = (R_i_f[0, 0] + R_i_f[1, 1] + R_i_f[2, 2] - 1) / 2
        inCos = np.clip(inCos, -1, 1)
        vf = np.arccos(inCos)
        if abs(vf) < 0.001:  # singularity for 180 degrees or 0 degrees
            rotMatrix = np.eye(4)
            rotMatrix[0:3, 0:3] = Ri
            quat = tf.transformations.quaternion_from_matrix(rotMatrix)
            return quat, np.zeros(3)

        r = (1/(2*np.sin(vf)))*np.array([[R_i_f[2, 1] - R_i_f[1, 2]],
                                         [R_i_f[0, 2] - R_i_f[2, 0]],
                                         [R_i_f[1, 0] - R_i_f[0, 1]]])
        # used for cubic interpolation
        v, dv = utils.calcPosVel(qi=np.array([0]),
                                 dqi=np.array([0]),
                                 qf=np.array([vf]),
                                 dqf=np.array([0]),
                                 tf=self.trajectory[self.index].pointTime,
                                 t=self.trajectoryTime)
        w_I = dv*r
        R_I = self.calcR_I(v, r)
        Re = Ri.dot(R_I)
        we = Ri.dot(w_I)
        rotMatrix = np.eye(4)
        rotMatrix[0:3, 0:3] = Re
        quat = tf.transformations.quaternion_from_matrix(rotMatrix)
        return quat, we.reshape((3,))

    def calcPointVel(self, p1, p2, p3, t2, t3):
        """ Calculates transitional velocities between trajectory parameters
            :param p1: position at point before
            :param p2: position at current point
            :param p3: position at point after
            :param t2: time between p1 and p2
            :param t3: time between p2 and p3
            :Returns a velocity for p2
        """
        vel = np.zeros(3)
        vk = (p2 - p1)/t2  # avg velocity for previous segment
        vkk = (p3 - p2)/t3  # avg velocity for next segment
        for i in range(3):
            # if velocity is close to zero for any part then the transition should have zeros velocity
            if np.abs(vk[i]) < 0.001 or np.abs(vkk[i]) < 0.001:  
                vel[i] = 0
                continue
            # avg vel if both parts have same velocity direction
            if np.sign(vk[i]) == np.sign(vkk[i]):
                vel[i] = 0.5*(vk[i] + vkk[i]) 
            else: # if the velocity change direction the transitional velocity is set to zero
                vel[i] = 0
        return vel

    def calcR_I(self, v, r):
        """ convert back to rotation matrix from axis an rotation
            :param v: rotation around axis
            :param r: axis of rotation
            :Returns rotation matrix"""
        cv = np.cos(v)[0]
        sv = np.sin(v)[0]
        rx = r[0, 0]
        ry = r[1, 0]
        rz = r[2, 0]
    
        R_I = np.array([[(rx**2 * (1-cv)+cv), (rx*ry*(1-cv) - rz*sv), (rx*rz*(1-cv)+ry*sv)],
                        [(rx*ry*(1-cv)+rz*sv), (ry**2 * (1-cv) + cv), (ry*rz*(1-cv)-rx*sv)],
                        [(rx*rz*(1-cv)-ry*sv), (ry*rz*(1-cv)+rx*sv), (rz**2 * (1-cv)+cv)]])
        return R_I