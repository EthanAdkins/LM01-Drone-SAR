import numpy as np
from stable_baselines3 import DQN
from stable_baselines3 import PPO
# from sb3_contrib import TRPO

import CustomEnv
import gymnasium as gym
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.monitor import Monitor
import Constants.configDrones as configDrones
LOCAL_IP = configDrones.LOCAL_IP
# from stable_baselines3 import DQN
# import gymnasium as gym
# from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
# from stable_baselines3.common.monitor import Monitor

if __name__ == '__main__':
    

    env = DummyVecEnv([lambda: Monitor(gym.make("airsim-drone-v0", LOCAL_IP=LOCAL_IP, Name = "0"))])
    env = VecTransposeImage(env)
    model_path = "/home/testuser/AirSim/PythonClient/multirotor/LM01-Drone-SAR/DroneSwarm/best_model.zip"
    print("Loading DQN Model")
    model = DQN.load(model_path)
    print("DQN Model Loaded")


    obs = env.reset()
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, done, info = env.step(action)
    print(action)
    if info[0]['goalreached'] == True:
            print("goal Reached")