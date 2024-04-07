import airsim
import numpy as np
import os
import gymnasium
from gymnasium import spaces
from PIL import Image, ImageStat
from matplotlib import pyplot as plt
import time
import random
from airsim.types import Pose,Quaternionr,Vector3r
import cv2
import threading
from threading import Thread
import glob
import os
import shutil
from decimal import Decimal

rewardConfig = {
    'collided': -100,
    'goal': 100,
    'timed': -100
}

TIME = 1
'''Static Environment for PPO: id=airsim-drone-continuous-v0'''
class drone_env_continuous(gymnasium.Env):
    def __init__(self):
        super(drone_env_continuous, self).__init__()
         #  -- set drone client --
        self.drone = airsim.MultirotorClient() 
        self.drone.confirmConnection()
        
        self.max_timestep= 600 
        # self.goal_position = np.array([50.0, 0.0, -2.8], dtype=np.float64) #Unreal object in cm 53
        
        self.step_length = 1
        self.img_width = 150
        self.img_height = 150
        self.elapsed = 0
        self.start_time = 0
        self.timestep_count = 0
        
        # self.goal_name = "Goal"
        self.goal_name = "character_2" #     this is the target (person) in Blocks
        goal_pos = self.drone.simGetObjectPose(self.goal_name).position
        self.goal_position = np.array([goal_pos.x_val, goal_pos.y_val, goal_pos.z_val], dtype=np.float64)

        self.goals = []
        self.sub_goal = 0
        # self.VertPos = []
        # self.HorzPos = []
        self.heading = Quaternionr(0, 0, 0, 0)

        # counter for idle movements
        self.no_movement = 0

        # these are 6 pre-determined locations to be used for placing the target randomly
        self.target_locations = np.array([[-132.19700622558594, -53.86787796020508, 22.765165328979492],
                            [68.42804718017578, -99.58522033691406, 1.0497558116912842],
                            [93.12804412841797, 75.0147705078125, -47.45024490356445],
                            [-111.7719497680664, 92.11477661132812, -29.95024299621582],
                            [-10.971952438354492, 113.81477355957031, -45.95024490356445],
                            [-8.071952819824219, -93.08522033691406, 15.249755859375]], dtype=np.float64)
        
        # -- set the Observation Space --
        self.observation_space = spaces.Dict({
            "image": spaces.Box(0, 255, shape=(self.img_height, self.img_width, 4), dtype=np.float64),
            "velocity": spaces.Box(low=np.array([-np.inf for _ in range(3)]), 
                                    high=np.array([np.inf for _ in range(3)]),
                                    dtype=np.float64),
            "prev_relative_distance": spaces.Box(low=np.array([-np.inf for _ in range(3)]), 
                                            high=np.array([np.inf for _ in range(3)]),
                                            dtype=np.float64),
            "relative_distance": spaces.Box(low=np.array([-np.inf for _ in range(3)]), 
                                            high=np.array([np.inf for _ in range(3)]),
                                            dtype=np.float64),
            "altimeter": spaces.Box(low=np.array([-np.inf for _ in range(1)]), 
                                    high=np.array([np.inf for _ in range(1)]),
                                    dtype=np.float64),
            })

        # -- set internally state and info --

        self.state = {
            "image": np.zeros([self.img_height , self.img_width, 1], dtype=np.uint8),
            "velocity": np.zeros(3),
            "prev_relative_distance": np.zeros(3),
            "relative_distance": np.zeros(3),
            "altimeter": np.zeros(1)
        }

        self.info = {
            "prev_image": np.zeros([self.img_height , self.img_width, 4], dtype=np.float64),
            "collision": False,
            "position": np.zeros(3),
            "prev_position": np.zeros(3),
            "goal_position": self.goal_position,
            "goalreached": False
        }

        """
        Create the continuous space:

        1 - Move Foward or Back
        2 - Move Right or Left
        3 - Move Up or Down
        4 - Rotate

        """
        self.action_space = spaces.Box(
            low=np.array([-1.0,-1.0,-1.0, -30.0]),
            high=np.array([1.0,1.0,1.0, 30.0]),
            dtype=np.float64
        )

        #  -- set drone client --
        # self.drone = airsim.MultirotorClient() 
        # self.drone.confirmConnection()

        #  -- set goal position --
        # position = Vector3r(self.goal_position[0], self.goal_position[1], self.goal_position[2])
        # pose = Pose(position, self.heading)
        # self.drone.simSetObjectPose(self.goal_name, pose, True)

        self.setGoals()
        
        # self.getParentObjPos()

        # -- set image request --
        # self.image_request = airsim.ImageRequest(
        #     "0", airsim.ImageType.DepthPerspective, True, False
        # )   
   
    
    # def getParentObjPos(self):
    #     VertNames = ["ParentVerticalFirstRow", "ParentVerticalSecondRow", "ParentVerticalThirdRow", "ParentVerticalFourthRow"]
    #     HorizNames = ["ParentHorizontalFirstRow", "ParentHorizontalSecondRow", "ParentHorizontalThirdRow", "ParentHorizontalFourthRow"]

    #     for x in range(len(VertNames)): #Assuming same size
    #         posV = self.drone.simGetObjectPose(VertNames[x])
    #         self.VertPos.append([posV, VertNames[x]])

    #         posH = self.drone.simGetObjectPose(HorizNames[x])
    #         self.HorzPos.append([posH, HorizNames[x]])
    #     self.orien = posH.orientation
        
    # def setGoals(self):
    #     distance, _ = self.get_distance()
    #     sub_distance = distance / 4
    #     print("Distance: " + str(distance))
    #     for _ in range(3):
    #         distance -= sub_distance
    #         self.goals.append(distance)
    #     self.goals.append(-99)
    #     print(self.goals)

    def setGoals(self):
        distance, _ = self.get_distance()
        self.original_distance = distance
        sub_distance = distance / 6
        print("Distance " + str(distance))
        print("Sub distance " + str(sub_distance))
        for _ in range(5):
            distance -= sub_distance
            self.goals.append(distance)
        self.goals.append(-99)
        print(self.goals)

    def doAction(self, action):
        
        # self.drone.startRecording()
        quad_vel = self.drone.getMultirotorState().kinematics_estimated.linear_velocity
        self.drone.moveByVelocityAsync(quad_vel.x_val + float(action[0]),
                                        quad_vel.y_val + float(action[1]),
                                        quad_vel.z_val + float(action[2]),
                                        0.5).join()
        
        self.drone.rotateByYawRateAsync(action[3], .5).join()

        print(f"move x {action[0]}, move y {action[1]}, move z {action[2]},rotate by {action[3]} degrees")    
        # self.drone.stopRecording()

        # return
    
    
    def get_distance(self):

        dist = np.linalg.norm(self.info["position"]-self.goal_position)
        prev_dist = np.linalg.norm(self.info["prev_position"]-self.goal_position)
        # print("current distance from goal: " + str(dist))
        # print("previous distance from goal: " + str(prev_dist))
       
        return dist, prev_dist
    
    def getRelativeDistance(self):

        r_x = np.linalg.norm(self.info["prev_position"][0]-self.goal_position[0])
        r_y = np.linalg.norm(self.info["prev_position"][1]-self.goal_position[1])
        r_z = np.linalg.norm(self.info["prev_position"][2]-self.goal_position[2])

        self.state["prev_relative_distance"] = np.array([r_x,r_y,r_z], dtype=np.float64)

        r_x = np.linalg.norm(self.info["position"][0]-self.goal_position[0])
        r_y = np.linalg.norm(self.info["position"][1]-self.goal_position[1])
        r_z = np.linalg.norm(self.info["position"][2]-self.goal_position[2])

        rel_dist = np.array([r_x,r_y,r_z], dtype=np.float64)
        return rel_dist
    
    #Punishes the drone for going farther than the original distance from the drone
    def radius_loss_eq(self, curr_dist_to_target):
        dist_change = curr_dist_to_target - self.original_distance
        loss = 25 / (1 + self.original_distance ** 2 * np.exp(-0.5 * dist_change))
        return loss

    def calculateReward(self, chosenAction): #figure out rewards
        done = False
        self.info["goalreached"] = False
        reward = 0

        # distance, previous_distance = self.get_distance()
        curr_distance, previous_distance = self.get_distance()
        # reward = (previous_distance - distance) - np.linalg.norm(self.info["prev_position"]-self.info["position"])

        # calculate euclidean distance between prev distance from goal and curr distance from goal, 
        # and subtract it with euclidean distance between previous agent position and current agent position
        # reward += (previous_distance - curr_distance) - np.linalg.norm(self.info["prev_position"]-self.info["position"])
        reward += (previous_distance - curr_distance) * 10
        print("original distance from Goal: " + str(self.original_distance))
        print("current distance from Goal: " + str(curr_distance))

        if curr_distance <= self.goals[self.sub_goal]:
            print("Level: "+str(self.sub_goal))
            self.sub_goal += 1 
            reward = 20
        
        # checking for idle action (hovering)
        if(Decimal('-0.2') <= Decimal(str(chosenAction[0])) <= Decimal('0.2') 
            and 
            Decimal('-0.2') <= Decimal(str(chosenAction[1])) <= Decimal('0.2')
            and 
            Decimal('-0.2') <= Decimal(str(chosenAction[2])) <= Decimal('0.2')):
            reward = -1.5
            self.no_movement+=1
        else: 
            self.no_movement = 0
        # check drone altimeter to make sure it's not too close or too high up the environment
        altimeter = self.state["altimeter"][0]
        if 5.0 <= altimeter <= 20.0:
            print(f"{altimeter} is within the range.")
            reward += 1
        else:
            print(f"{altimeter} is out the range.")
            reward -= 0.5

        if self.info["collision"]:
            reward -= 150
            print("System: Drone collision.")
            done = True

        if curr_distance < 10.0:
        # if self.state["relative_distance"][0] < 2:
            reward += 400
            self.info["goalreached"] = True
            print("System: Goal Reached.")
            done = True

        if self.no_movement == 10:
            print(f"System: Drone had not moved after {self.no_movement} actions")
            reward -= 50
            done = True

        if self.timestep_count > self.max_timestep:
            print("System: Time Step Limit Reached.")
            reward -= 100
            done = True
        
        targetdistance_threshold = 50
        if curr_distance >= self.original_distance + targetdistance_threshold:
            print("System: Drone is TOO far from target.")
            reward -= 50
            done = True

        print("Final reward: "+str(reward))
        print("\n")
        #print(reward)
        
        return reward, done
    
    def step(self, chosenAction):
        self.timestep_count += 1
        self.drone.simPause(False)

        self.doAction(chosenAction)
        
        # obs = self.getObservation()
        obsAq = self.getObservation()
        obs = obsAq[0]

        reward, terminated = self.calculateReward(chosenAction)

        #  -- Sometimes image bounces over obstacles once collision triggers --
        if terminated:
           mean1 = np.mean(self.state["image"])
           mean2 = np.mean(self.info["prev_image"])

           if mean2 > mean1:
            self.state["image"] = self.info["prev_image"]

        info = self.info

        return obs, reward, terminated, False, info

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.seed(seed)
        self.timestep_count = 0
        self.sub_goal = 0

        #  -- Reset our action history --
        # self.state["action_history"] = -1 * np.ones(10, dtype=np.int8)
        self.drone.simPause(False)
        self.startFlight()
        self.drone.simPause(True)

        # reset counter for idle movements
        self.no_movement = 0

        return self.getObservation()
    
    def randomiseTarget(self):
        locpick = random.randint(0,5)
        target_x = self.target_locations[locpick, 0]
        target_y = self.target_locations[locpick, 1]
        target_z = self.target_locations[locpick, 2]
        print(f"target location randomized: {target_x}, {target_y}, {target_z}")
        heading = Quaternionr(0, 0, 0, 0)
        position = Vector3r(target_x, target_y, target_z)
        pose = Pose(position, heading)
        self.drone.simSetObjectPose("character_2", pose, True)
        # update new goal_position for new target location
        self.goal_position = np.array([target_x, target_y,target_z], dtype=np.float64)
        self.info["goal_position"] = self.goal_position


    def startFlight(self):

        self.drone.reset()
        
        #  -- Randomise our drones starting location --
        ry = random.uniform(-9,9)
        rz = random.uniform(-7,0)

        position = airsim.Vector3r(0, ry, rz)
        heading = airsim.utils.to_quaternion(0, 0, 0)
        pose = airsim.Pose(position, heading)
        self.drone.simSetVehiclePose(pose, True)
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True) 

        # update self.info["position"] starting point
        self.info["position"] = np.array([0,ry,rz], dtype=np.float64)
        print("drone starting position: ")
        print(self.info["position"])
        #  -- Randomise target location and set new sub goals target --
        self.randomiseTarget()
        self.goals.clear()
        self.original_distance = 0
        self.setGoals()

        print("System update: Drone has successfully been reset")
        # self.drone.startRecording()
        self.drone.moveByVelocityAsync(0, 0, 0, 10).join()
        # self.drone.stopRecording()

    
    def getImageObs(self):
        my_path = "F:/Documents/RLModel_Pics/"
        # -- set image request --
        image_request = self.drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False), 
                                                    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)])
        request = image_request
        # get depth image
        depth_img_in_meters = airsim.list_to_2d_float_array(request[0].image_data_float, request[0].width, request[0].height)
        depth_img_in_meters = depth_img_in_meters.reshape(request[0].height, request[0].width, 1)
        depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))
        if depth_8bit_lerped.size != (150*150*1):
            print("Returned bad depth image data, creating dummy array to prevent training from stopping")
            depth_test = np.zeros(150*150)
            depth_8bit_lerped = np.reshape(depth_test,(150, 150, 1))

        # get rgb image
        rgb = np.frombuffer(request[1].image_data_uint8, dtype=np.uint8)
        rgb_2d = np.reshape(rgb, (request[1].height, request[1].width, 3))
        if rgb_2d.size != (150*150*3):
            print("Returned bad rgb image data, creating dummy array to prevent training from stopping")
            rgb_test = np.ones(150*150*3)
            rgb_2d = np.reshape(rgb_test,(150, 150,3))
        
        # get rgb-d image
        rgb_d = np.concatenate((rgb_2d, depth_8bit_lerped), axis=-1)

        # sanity check
        # airsim.write_png(os.path.normpath(f'{my_path}/imageStacked.png'), rgb_d)
        return rgb_d

    def getObservation(self):

        self.drone.simPause(True)

        image = self.getImageObs()
        # image_path = "C:/Users/andre/Desktop/ThesisUnReal/TestImages2/"
        # airsim.write_png(os.path.normpath(f'{image_path}/imageChanged.png'), image)
        self.info["prev_image"] = self.state["image"] 
        self.state["image"] = image

        self.drone_state = self.drone.getMultirotorState()

        kinematics = self.drone.getMultirotorState().kinematics_estimated
        self.info["collision"] = self.drone.simGetCollisionInfo().has_collided #check if drone has collided

        v_x = kinematics.linear_velocity.x_val
        v_y = kinematics.linear_velocity.y_val
        v_z = kinematics.linear_velocity.z_val
        self.state["velocity"] = np.array([v_x,v_y,v_z], dtype=np.float64)

        p_x = kinematics.position.x_val
        p_y = kinematics.position.y_val
        p_z = kinematics.position.z_val
        self.info["prev_position"] = self.info["position"] #get previous position
        self.info["position"] = np.array([p_x,p_y,p_z], dtype=np.float64) #get current position

        self.state["relative_distance"] = self.getRelativeDistance()

        get_alt = self.drone.getDistanceSensorData("Distance", "Drone1").distance
        self.state["altimeter"] = np.array([get_alt], dtype=np.float64)

        # return self.state
        obs = self.state
        info = self.info

        return obs, info

    
    def disconnect(self):
        self.drone.enableApiControl(False)
        self.drone.armDisarm(False)
        print('System: Disconnected.')

        return