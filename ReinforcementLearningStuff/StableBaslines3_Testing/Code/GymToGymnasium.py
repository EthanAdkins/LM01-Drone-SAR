import numpy as np
from stable_baselines3 import DQN
from stable_baselines3 import PPO
# from sb3_contrib import TRPO

import CustomEnv
import gymnasium
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.monitor import Monitor

'''Saving models trained in Gym to a Gymnasium compatible version'''
# Load the evaluation metrics from 'evaluations.npz' file
# path = "C:/Users/User/Desktop/ThesisUnReal/CheckPoints/PPO/BestModel/Best-Model-PPO-StaticFinal/"
#path = "C:/Users/User/Desktop/ThesisUnReal/CheckPoints/DQN/BestModel/Best-Model-DQN-FinalDynamic/"
# path = "C:/Users/User/Desktop/ThesisUnReal/CheckPoints/TRPO/BestModel/Best-Model-TRPO-StaticFinal/"

# path = "C:/Users/andre/Documents/GitHub/LM01-Drone-SAR/ReinforcementLearningStuff/StableBaslines3_Testing/Code/CheckPoints/DQN/BestModel_PT/Best-Model-DQN-FinalStatic/"
# path = "C:/Users/andre/Documents/GitHub/LM01-Drone-SAR/ReinforcementLearningStuff/StableBaslines3_Testing/Code/CheckPoints/PPO/BestModel/Best-Model-PPO-StaticFinal/"
# path = "C:/Users/andre/Documents/GitHub/LM01-Drone-SAR/ReinforcementLearningStuff/StableBaslines3_Testing/Code/CheckPoints/DQN/BestModel/"


path = "C:/Users/andre/Desktop/ThesisUnReal/Best-Model-DQN-FinalStatic/"
# Load the saved model parameters
model = DQN.load(path + 'best_model.zip')
# model = PPO.load(path + 'best_model.zip')
# model = TRPO.load(path + 'best_model.zip')

model.save("C:/Users/andre/Desktop/ThesisUnReal/Best-Model-DQN-Static-Gymnasium/best_model")