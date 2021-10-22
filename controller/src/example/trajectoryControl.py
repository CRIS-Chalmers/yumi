#!/usr/bin/env python3

import rospy

import os, sys
print(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Controller.controller import YumiController
from Controller.controlTarget import ControlTarget
from parameters import Paramters
from controller.msg import Trajectory_msg
import Controller.utils as utils
import numpy as np

class TrajectoryController(YumiController):
    def __init__(self):
        super(TrajectoryController, self).__init__()
        #self.initialPose # if not set default will be used. 
        self.controlTarget = ControlTarget(Paramters.dT)
        self.reset = False

    def policy(self):
        # resets yumi to init pose
        if self.reset:
            self.reset = resetPose(self.targetResetPose)
            return
            
        action = dict()

       # Update the pose for controlTarget class  
        self.controlTarget.updatePose(yumiGrippPoseR=self.yumiGrippPoseR,\
                                                 yumiGrippPoseL=self.yumiGrippPoseL)

        # calculates target velocities and positions
        self.controlTarget.updateTarget()

        if self.controlTarget.checkNewTrajectorySegment():
            action['gripperRight'] = self.controlTarget.gripperRight
            action['gripperLeft'] = self.controlTarget.gripperLeft 
            
        action['controlSpace'] = self.controlTarget.mode

        # k is the gain for vel = vel + k*error
        if self.controlTarget.mode == 'individual':
            action['cartesianVelocity'] = self.controlTarget.getIndividualTargetVelocity(k_p=Paramters.k_p_i, k_o=Paramters.k_o_i)
        elif self.controlTarget.mode == 'coordinated':
            action['absoluteVelocity'] = self.controlTarget.getRelativeTargetVelocity(k_p=Paramters.k_p_r, k_o=Paramters.k_o_r) 
            action['relativeVelocity'] = self.controlTarget.getAbsoluteTargetVelocity(k_p=Paramters.k_p_a, k_o=Paramters.k_o_a) 
        action['colisionAvoidance'] = False
        self.setAction(action)


    def callbackTrajectory(self, data):
        # Gets called when a new set of trajectory paramters is recived
        # The variable names in this funciton and the the trajectory class follows 
        # individual motion with left and right . This means when coordinate manipulation 
        # is usd, right is aboslute motion and left becomes relative motion. 
        
        # get current pose, used as first trajectory paramter 
        if data.mode == 'coordinated':
            positionRight = np.copy(self.controlTarget.absolutePosition)
            positionLeft  = np.copy(self.controlTarget.realativPosition) 
            orientationRight = np.copy(self.controlTarget.absoluteOrientation)
            orientationLeft = np.copy(self.controlTarget.rotationRelative)
        elif data.mode == 'individual':
            positionRight = np.copy(self.controlTarget.translationRightArm)
            positionLeft  = np.copy(self.controlTarget.translationLeftArm)
            orientationRight = np.copy(self.controlTarget.rotationRightArm)
            orientationLeft = np.copy(self.controlTarget.rotationLeftArm)
        else:
            print('Error, mode not matching combined or individual')
            return
        # current gripper position 
        gripperLeft = self.controlTarget.gripperLeft
        gripperRight = self.controlTarget.gripperRight
        # add current state as a trajectory point
        currentPoint = utils.TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight)
        trajectory = [currentPoint]

        # append trajectory points from msg
        for i in range(len(data.trajectory)):
            if data.mode == 'coordinated':
                positionRight = np.asarray(data.trajectory[i].positionAbsolute)
                positionLeft  = np.asarray(data.trajectory[i].positionRelative)
                orientationRight = np.asarray(data.trajectory[i].orientationAbsolute)
                orientationLeft = np.asarray(data.trajectory[i].orientationRelative)
            else:
                positionRight = np.asarray(data.trajectory[i].positionRight)
                positionLeft  = np.asarray(data.trajectory[i].positionLeft)
                orientationRight = np.asarray(data.trajectory[i].orientationRight)
                orientationLeft = np.asarray(data.trajectory[i].orientationLeft)

            gripperLeft = data.trajectory[i].gripperLeft
            gripperRight = data.trajectory[i].gripperRight
            pointTime = np.asarray(data.trajectory[i].pointTime)
            trajectroyPoint = utils.TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight,\
                                                    pointTime=pointTime)
            trajectory.append(trajectroyPoint)

        #self.lock.acquire() # lock when updating trajectory       

        # use current velocity for smoother transitions, (only for translation)
        if self.controlTarget.mode == data.mode: # no change in control mode
            velLeftInit = np.copy(self.controlTarget.targetVelocities[6:9])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        elif self.controlTarget.mode == 'individual': # going from individual to coordinate motion
            # simple solution, not fully accurate trasition 
            velLeftInit = np.zeros(3)
            velRightInit = 0.5*(np.copy(self.controlTarget.targetVelocities[0:3]) +\
                                     np.copy(self.controlTarget.targetVelocities[6:9]))
        elif self.controlTarget.mode == 'coordinated': # going from coordinated to individual motion
            # simple solution, not fully accurate trasition 
            velLeftInit = np.copy(self.controlTarget.targetVelocities[0:3])
            velRightInit = np.copy(self.controlTarget.targetVelocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)
            
        # set mode
        self.controlTarget.mode = data.mode

        # update the trajectroy 
        self.controlTarget.trajectory.updateTrajectory(trajectory, velLeftInit, velRightInit)
        self.controlTarget.trajIndex = 0
        self.controlTarget.trajectorySegment = 0

        #self.lock.release() 



def main():
    # starting ROS node and subscribers
    rospy.init_node('controller', anonymous=True) 

    ymuiContoller = TrajectoryController()
    rospy.Subscriber("/Trajectroy", Trajectory_msg, ymuiContoller.callbackTrajectory, queue_size=1, tcp_nodelay=True)

    rospy.spin()

if __name__ == '__main__':
    main()