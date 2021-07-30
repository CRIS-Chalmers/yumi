#!/usr/bin/env python3

# this controller meant to put the robot in a good initial position, both for repeatability and 
# for avoiding singularities or joint saturation that may occur from a bad intial configuration 
# when using inverse kinematics controllers. 
import rospy
import numpy as np
from controller.msg import Jacobian_msg
from std_msgs.msg import Float64MultiArray, Float64, Int64
import utils 

class InitContoller(object):
    def __init__(self):
        self.updateRate = 50 #Hz also defined in kdl_jacobian 
        self.dT = 1/self.updateRate
        self.jointState = utils.JointState()
        self.calibrationJointPosition = np.array([0, -130, -135, 30, 0, 40, 0,\
             0, -130, 135, 30, 0, 40, 0])*np.pi/180
        self.calTeast = np.array([78.25, -94.37, -82.12, 40.59, -150.00, 86.26, 56.28,
                                -78.25, -94.37, 82.12, 40.59, 150.00, 86.26, -56.28])*np.pi/180
        self.finalJointPosition = np.array([0.7, -1.7, -0.8, 1.0, -2.2, 1.0, 0.0, -0.7, -1.7, 0.8, 1.0, 2.2, 1.0, 0.0]) # need to be tuned
        #self.finalJointPosition = self.calibrationJointPosition
        #self.finalJointPosition = self.calTeast
        self.startJointPosition = np.zeros(14) 
        self.k = 0 # positon error gain  
        self.firstDataPoint = 0
        self.finalTime = 10 # updates later
        self.minTime = 2
        self.maxAngleVelocity = 3 * np.pi/180
        self.time = 0
        self.pub = rospy.Publisher('/yumi/egm/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)

    def callback(self, data):

        self.jointState.UpdatePose(pose=np.asarray(data.jointPosition))
        
        self.time += self.dT

        if self.firstDataPoint == 0:
            jointState = self.jointState.GetJointPosition()[0:14]
            diff = self.finalJointPosition - jointState
            maxErrorAngle = max(diff)
            timeMax = maxErrorAngle/self.maxAngleVelocity
            self.finalTime = max(timeMax, self.minTime)
            self.startJointPosition = jointState
            self.firstDataPoint = 1

        if self.time > self.finalTime:
            self.jointState.UpdateVelocity(np.zeros(14))
            self.publishVelocity()
            print('Init done')
            rospy.sleep(0.5)
            rospy.signal_shutdown("done")
       
        q, dq = self.calcPosVel(self.startJointPosition[0:14], np.zeros(14),\
                    self.finalJointPosition[0:14], np.zeros(14), self.finalTime, self.time)

        vel = dq + self.k*(q-self.jointState.GetJointPosition()[0:14])
        self.jointState.UpdateVelocity(vel)

        self.publishVelocity()



    def publishVelocity(self):
        msg = Float64MultiArray()
        # Left Arm, Right Arm
        msg.data = np.hstack([self.jointState.GetJointVelocity()[7:14], self.jointState.GetJointVelocity()[0:7]]).tolist()
        self.pub.publish(msg)


    def calcPosVel(self, qi, dqi, qf, dqf, tf, t): # outputs target position and velocity 
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


def main():

    # starting ROS node and subscribers
    rospy.init_node('InitController', anonymous=True) 

    initContoller = InitContoller()
    rospy.sleep(0.05)
    rospy.Subscriber("/Jacobian_R_L", Jacobian_msg, initContoller.callback, queue_size=3)

    rospy.spin()

if __name__ == '__main__':
    main()