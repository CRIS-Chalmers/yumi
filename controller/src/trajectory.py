#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import utils

class Trajectory(object):
    # Generates a trajectory from trajectory paramters, the process is identical for individual and coordinated manipulation. 
    # The variable names follows individual motion with left and right. This means when coordinate manipulation is usd, right 
    # is aboslute motion and left becomed relative motion. 
    def __init__(self, deltaTime):
        self.trajectory = [] # stores trajectory paramters
        self.trajectoryTime = 0 # keeps track on time, resest for every trajectory segment
        self.deltaTime = deltaTime
        self.index = 1 # current target trajectory parameter, 0 is the current position. 
        self.numberOfTrajectoryParamters = 0
        self.positionVelocitiesLeft = [] # stores velocity transitions between trajectory paramters
        self.positionVelocitiesRight = []
        self.roationMatrixLeft = [] # stores rotation matrices to save some computation
        self.roationMatrixRight = []
        self.desierdVelocity = np.zeros(12) # desierd velocity from trajectory
        self.targetPosition = np.zeros(6) # cooresponiding target positon and orientation
        self.targetOrientation = np.zeros(8)
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

    def updateTrajectory(self, trajcetoryParamters, velLeftInit, velRightInit):
        # updates the trajectory when a new trajectory as been recived

        self.positionVelocitiesLeft = [velLeftInit] # current velocity of the robot 
        self.positionVelocitiesRight = [velRightInit]
        self.roationMatrixLeft = [] # reset rotation matrices
        self.roationMatrixRight = []
        # reset paramters 
        self.index = 1 
        self.trajectoryTime = 0
        self.trajectory = trajcetoryParamters
        self.numberOfTrajectoryParamters = len(self.trajectory)

        # calculate list of rotation matrices for each trajectory paramters 
        for i in range(0,self.numberOfTrajectoryParamters):
            tfMatrixRight = self.transformer.fromTranslationRotation(translation=np.zeros(3), rotation=trajcetoryParamters[i].orientationRight)
            tfMatrixLeft = self.transformer.fromTranslationRotation(translation=np.zeros(3), rotation=trajcetoryParamters[i].orientationLeft)
            self.roationMatrixLeft.append(tfMatrixLeft[0:3,0:3])
            self.roationMatrixRight.append(tfMatrixRight[0:3,0:3])

        # calculate the transition velocities between the trajectory paramters. 
        for i in range(1,self.numberOfTrajectoryParamters-1):
            vel = self.calcPointVel(trajcetoryParamters[i-1].positionRight, trajcetoryParamters[i].positionRight,\
                 trajcetoryParamters[i+1].positionRight, trajcetoryParamters[i].pointTime, trajcetoryParamters[i+1].pointTime)
            self.positionVelocitiesRight.append(vel)
            vel = self.calcPointVel(trajcetoryParamters[i-1].positionLeft, trajcetoryParamters[i].positionLeft,\
                 trajcetoryParamters[i+1].positionLeft, trajcetoryParamters[i].pointTime, trajcetoryParamters[i+1].pointTime)
            self.positionVelocitiesLeft.append(vel)

        # last point always has velocity 0.
        self.positionVelocitiesRight.append(np.zeros(3))
        self.positionVelocitiesLeft.append(np.zeros(3))


    def getTarget(self):
        # calculates the desierd velocities and target pose. 

        # upates the current target trajectory paramter or which is the curent segment on the trajectory
        if self.trajectory[self.index].pointTime < self.trajectoryTime:
            if self.index < self.numberOfTrajectoryParamters - 1:
                self.trajectoryTime = 0
                self.index += 1 
            else: # for last point 
                self.trajectoryTime = self.trajectory[self.index].pointTime
                self.index = self.numberOfTrajectoryParamters - 1

        # calculates the target position and desierd velocity for a time point
        q, dq = self.calcPosVel(qi=self.trajectory[self.index-1].positionRight,\
                                    dqi=self.positionVelocitiesRight[self.index-1],\
                                    qf=self.trajectory[self.index].positionRight,\
                                    dqf=self.positionVelocitiesRight[self.index],\
                                    tf=self.trajectory[self.index].pointTime,\
                                    t=self.trajectoryTime)
        self.targetPosition[0:3] = q
        self.desierdVelocity[0:3] = dq

        q, dq = self.calcPosVel(qi=self.trajectory[self.index-1].positionLeft,\
                                    dqi=self.positionVelocitiesLeft[self.index-1],\
                                    qf=self.trajectory[self.index].positionLeft,\
                                    dqf=self.positionVelocitiesLeft[self.index],\
                                    tf=self.trajectory[self.index].pointTime,\
                                    t=self.trajectoryTime)
        self.targetPosition[3:6] = q
        self.desierdVelocity[6:9] = dq

        # calculates the target orientation and desierd angular velocity for a time point
        quat, we = self.calcOrientation(Ri=self.roationMatrixRight[self.index-1],\
                                Rf=self.roationMatrixRight[self.index])
        self.targetOrientation[0:4] = quat
        self.desierdVelocity[3:6] = we

        quat, we = self.calcOrientation(Ri=self.roationMatrixLeft[self.index-1],\
                                Rf=self.roationMatrixLeft[self.index])
        self.targetOrientation[4:8] = quat
        self.desierdVelocity[9:12] = we

        # update time 
        self.trajectoryTime += self.deltaTime

        return self.targetPosition, self.targetOrientation, self.desierdVelocity,\
             self.trajectory[self.index].gripperLeft, self.trajectory[self.index].gripperRight

    def calcPosVel(self, qi, dqi, qf, dqf, tf, t): 
        # outputs target position and velocity, cubic interpolation between points 
        num = np.shape(qi)[0]
        q = np.zeros(num)
        dq = np.zeros(num)
        for k in range(num):
            a0 = qi[k]
            a1 = dqi[k]
            a2 = 3 * (qf[k] - (dqf[k]*tf)/3 - a1*tf*(2/3) - a0)/(tf*tf)
            a3 = (dqf[k] - (2*a2*tf + a1))/(3*tf*tf)
            q[k] = a3*t**3 + a2*t**2  + a1*t + a0
            dq[k] = 3*a3*t**2 + 2*a2*t + a1
        return q, dq

    def calcOrientation(self, Ri, Rf): 
        # outputs target orientation and angular velocity
        # cubic interpolation betweem points, but alwasys zero transition velocity 
        R_i_f = np.transpose(Ri).dot(Rf)
        inCos = (R_i_f[0,0] + R_i_f[1,1] + R_i_f[2,2] - 1) / 2
        inCos = np.clip(inCos,-1,1)
        vf = np.arccos(inCos)
        if abs(vf) < 0.001: # sigularity for 180 degrees or 0 degrees
            rotMatrix = np.eye(4)
            rotMatrix[0:3,0:3] = Ri 
            quat = tf.transformations.quaternion_from_matrix(rotMatrix)
            return quat, np.zeros(3)

        r = (1/(2*np.sin(vf)))*np.array([[R_i_f[2,1] - R_i_f[1,2]],\
                                    [R_i_f[0,2] - R_i_f[2,0]],\
                                    [R_i_f[1,0] - R_i_f[0,1]]])
        # used for cubic interpolation
        v, dv = self.calcPosVel(qi=np.array([0]),\
                                    dqi=np.array([0]),\
                                    qf=np.array([vf]),\
                                    dqf=np.array([0]),\
                                    tf=self.trajectory[self.index].pointTime,\
                                    t=self.trajectoryTime)
        w_I = dv*r
        R_I = self.calcR_I(v, r)
        Re = Ri.dot(R_I)
        we = Ri.dot(w_I)
        rotMatrix = np.eye(4)
        rotMatrix[0:3,0:3] = Re 
        quat = tf.transformations.quaternion_from_matrix(rotMatrix)
        return quat, we.reshape((3,))

    def calcPointVel(self, v1, v2, v3, t2, t3): 
        # Calculates trasitional velocities between trajectory parameters
        vel = np.zeros(3)
        vk = (v2 - v1)/t2 # avg velocity for previous segment 
        vkk = (v3 - v2)/t3 # avg velocity for next segment 
        for i in range(3):
            # if velocity is close to zero for any part then the transition should have zeros veloicty
            if np.abs(vk[i]) < 0.001 or np.abs(vkk[i]) < 0.001:  
                vel[i] = 0
                continue
            # avg vel if both parts have same velocity direction
            if np.sign(vk[i]) == np.sign(vkk[i]):
                vel[i] = 0.5*(vk[i] + vkk[i]) 
            else: # if the velocity change direction the transitional velocity is set to zero
                vel[i] = 0
        return vel

    def calcR_I(self, v, r): # convert back to rotation matrix from axis an rotation
        cv = np.cos(v)[0]
        sv = np.sin(v)[0]
        rx = r[0,0]
        ry = r[1,0]
        rz = r[2,0]
    
        R_I = np.array([[(rx**2 * (1-cv)+cv), (rx*ry*(1-cv)- rz*sv), (rx*rz*(1-cv)+ry*sv)],\
                 [(rx*ry*(1-cv)+rz*sv),(ry**2 * (1-cv) + cv),(ry*rz*(1-cv)-rx*sv)],\
                  [(rx*rz*(1-cv)-ry*sv),(ry*rz*(1-cv)+rx*sv),(rz**2 * (1-cv)+cv)]])
        return R_I