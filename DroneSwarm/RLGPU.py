import Constants.ros as ros
import rospy
import os
import torch
import numpy as np
import pandas
import cv2
from airsim_ros_pkgs.srv import requestRLGPU
# Import SB3
from stable_baselines3 import DQN
import gymnasium as gym
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.monitor import Monitor

global MODEL

# Environmental Variables
# ros: services
RLGPU_SERVICE = ros.RLGPU_SERVICE
isModelLoaded = False

# Main Process Start ----------------------------------------------
def startRLGPU():
    global isModelLoaded
    global MODEL
    print("StartingRLGPU")
    # Create node for "ProximityWolf"
    nodeName = "RLGPU"
    rospy.init_node(nodeName, anonymous = True)

    # start yolo GPU
    cwd = os.getcwd()
    rlPT = os.path.join(str(cwd), 'best_model')

    # Model is loaded global to be used by service functions
    try:
        print("Loading RL model")
        MODEL = DQN.load(rlPT)
        print("RL Model loaded")
        isModelLoaded = True
    except:
        isModelLoaded = False

    # Spins up gpu service
    print("Starting RL gpu service")
    startRLGPUService()

    # Main Process end -----------------------------------------------

# Starts of gpu service for handling requests, requests are then done through the handleGPU function
def startRLGPUService():
    serviceName = RLGPU_SERVICE
    service = rospy.Service(serviceName, requestRLGPU, handleRLGPU)
    rospy.spin()

def handleRLGPU(request):
    global isModelLoaded
    global MODEL

    responseString = request.responseString

    # Send back gpu model status if reponse string is empty
    if (responseString == ""):
        return (False, 0, 0, 0, 0, 0)
    else:
        
        return (False, 0, 0, 0, 0, 0)
        # load RL model

