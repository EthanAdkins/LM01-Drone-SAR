import numpy as np
from stable_baselines3 import DQN
from stable_baselines3 import PPO
# from sb3_contrib import TRPO

import CustomEnv
import gymnasium as gym
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.monitor import Monitor


'''Continuous (for PPO)'''
# env = DummyVecEnv([lambda: Monitor(gym.make("airsim-drone-continuous-v0"))])
'''Discrete (for DQN)'''
#env = DummyVecEnv([lambda: Monitor(gym.make("airsim-drone-dynamic-v0"))])
env = DummyVecEnv([lambda: Monitor(gym.make("airsim-drone-v0"))])

env = VecTransposeImage(env)


# Load the evaluation metrics from 'evaluations.npz' file
# path = "C:/Users/User/Desktop/ThesisUnReal/CheckPoints/PPO/BestModel/Best-Model-PPO-StaticFinal/"
#path = "C:/Users/User/Desktop/ThesisUnReal/CheckPoints/DQN/BestModel/Best-Model-DQN-FinalDynamic/"
# path = "C:/Users/User/Desktop/ThesisUnReal/CheckPoints/TRPO/BestModel/Best-Model-TRPO-StaticFinal/"
path = "C:/Users/andre/Documents/GitHub/LM01-Drone-SAR/ReinforcementLearningStuff/StableBaslines3_Testing/Code/CheckPoints/DQN/BestModel_PT/Best-Model-DQN-FinalStatic/"
# path = "C:/Users/andre/Documents/GitHub/LM01-Drone-SAR/ReinforcementLearningStuff/StableBaslines3_Testing/Code/CheckPoints/DQN/BestModel/"

# Load the saved model parameters
model = DQN.load(path + 'best_model.zip')
#model = PPO.load(path + 'best_model.zip')
# model = TRPO.load(path + 'best_model.zip')
runs = 100
# Use the model to make predictions
goal = 0
for ep in range(runs):
    print('Run: '+str(ep))
    done = False
    obs = env.reset()
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        # print(info)
        if info[0]['goalreached'] == True:
            goal += 1

print('Percentage of successful runs '+str(goal/runs * 100))

        
