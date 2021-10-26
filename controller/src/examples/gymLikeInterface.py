#!/usr/bin/env python3

import rospy
from controller import YmuiController
from gym_utils import ObservationConfig, ActionConfig, RewardConfig

def policy(observation):
    return np.ones(14)

class GymLike(YmuiContoller):
    def __init__(self):
        super(GymLike, self).__init__()

        observationConfig = ObservationConfig()
        actionConfig = ActionConfig()
        rewardConfig = RewardConfig()
        self.reset = True
        # velocity control, three different spaces
        self.controlSpace = 'jointSpace' # 'jointSpace', 'individual' or 'coordinated'

        #self.targetResetPose # if not set default will be used. 

    def policy():
        # resets yumi to init pose
        if self.reset:
            self.reset = self.resetPose(self.targetResetPose)
            return

        # observation config for modifying the observation, also used for combining 
        # other observations such as camera stream with kwargs agruments, e.g. observation = observationConfig.observation('yumi_obs'=yumi_observation, 'rgb_img'=image)
        observation = observationConfig.observation()
        
        # replace policy 
        _action = policy(observation)

        # action config for modifying the poliy action before sending to robot
        action = actionConfig.action('action'=_action, 'control_space'=self.controlSpace)

        # calculates reward. 
        reward, done, info = rewardConfig.calcReward('observation'=observation, 'action'=action)

        if done:
            self.reset = True
            return
        
        self.setAction(action)

    def resetPose(observation):
        pass
        # TODO implement reset funciton 


def main():
    # starting ROS node and subscribers
    rospy.init_node('controller', anonymous=True) 

    ymuiContoller = GymLike()

    rospy.spin()

if __name__ == '__main__':
    main()