import airsim
import random
import glob
import time
import cv2
import numpy as np
import time
import os
import shutil
from airsim.types import Pose,Quaternionr,Vector3r
#import warnings
#warnings.filterwarnings("ignore")


        



collision = False
drone = airsim.MultirotorClient() 
drone.confirmConnection()
drone.simPause(False)
ry = random.uniform(-9,9)
rz = random.uniform(-7,0)

print(f"Setting initial position to 0, {ry}, {rz}")
position = airsim.Vector3r(0, ry, rz)
heading = airsim.utils.to_quaternion(0, 0, 0)
pose = airsim.Pose(position, heading)
drone.simSetVehiclePose(pose, True)
drone.enableApiControl(True)
drone.armDisarm(True) 
drone.moveByVelocityAsync(0, 0, 0, 3).join()

# my_path = "F:/Documents/RLModel_Pics/"

# for filename in os.listdir(my_path):
#     file_path = os.path.join(my_path, filename)
#     try:
#         if os.path.isfile(file_path) or os.path.islink(file_path):
#             os.unlink(file_path)
#         elif os.path.isdir(file_path):
#                 shutil.rmtree(file_path)
#     except Exception as e:
#         print('Failed to delete %s. Reason: %s' % (file_path, e))

print(drone.simListSceneObjects())
num = 0
imagestack=[]
my_path = "F:/Documents/RLModel_Pics/"

target_locations = np.array([[-132.19700622558594, -53.86787796020508, 22.765165328979492],
                            [68.42804718017578, -99.58522033691406, 1.0497558116912842],
                            [93.12804412841797, 75.0147705078125, -47.45024490356445],
                            [-111.7719497680664, 92.11477661132812, -29.95024299621582],
                            [-10.971952438354492, 113.81477355957031, -45.95024490356445],
                            [-8.071952819824219, -93.08522033691406, 15.249755859375]], dtype=np.float64)
print("target initial position: ")
print(drone.simGetObjectPose("character_2").position)
print("\n")

for _ in range(3):
# --------------------------------------------------------------------------------------
    
  
    # isExist = os.path.exists(my_path)
    # if not isExist:
    # # Create a new directory because it does not exist
    #     os.makedirs(my_path)
    # #print("The new directory is created!")

    # drone.startRecording()
    #drone.simPause(False)
    drone.moveByVelocityAsync(1, 0, 0, 1).join()
    #drone.simPause(True)

    start_time = time.time()
    # drone.stopRecording()

    # --------------------------------------------------------------------------------------
    
    # --------------------------------------------------------------------------------------
    start_time = time.time()
    image_request = drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False), 
                                                    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)])

    response_depth = image_request[0]
    depth_img_in_meters = airsim.list_to_2d_float_array(response_depth.image_data_float, response_depth.width, response_depth.height)
    depth_img_in_meters = depth_img_in_meters.reshape(response_depth.height, response_depth.width, 1)
    depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))
    airsim.write_png(os.path.normpath(f'{my_path}/imageChangedDepth{num}.png'), depth_8bit_lerped)
    
    # ---------------------------------RGB Capture Picture-----------------------------------------------------
    response_rgb = image_request[1]
    rgb = np.frombuffer(response_rgb.image_data_uint8, dtype=np.uint8)
    rgb_2d = np.reshape(rgb, (response_rgb.height, response_rgb.width, 3))
    airsim.write_png(os.path.normpath(f'{my_path}/imageChangedRGB{num}.png'), rgb_2d)
    rgb_d = np.concatenate((rgb_2d, depth_8bit_lerped), axis=-1)
    print(rgb_d.shape)
    airsim.write_png(os.path.normpath(f'{my_path}/imageStacked{num}.png'), rgb_d)
    # --------------------------------------------------------------------------------------
    # collision = drone.simGetCollisionInfo().has_collided
    # # print(collision)
    # # if collision:
    # drone.simPause(False)
    # collision = False

    distance_sensor_data = drone.getDistanceSensorData("Distance", "Drone1")
    print(distance_sensor_data)
    

    drone.reset()
    
    #  -- Randomise our drones starting location --
    ry = random.uniform(-9,9)
    rz = random.uniform(-7,0)

    print(f"Setting new position to 0, {ry}, {rz}")
    position = airsim.Vector3r(0, ry, rz)
    heading = airsim.utils.to_quaternion(0, 0, 0)
    pose = airsim.Pose(position, heading)
    drone.simSetVehiclePose(pose, True)
    drone.enableApiControl(True)
    drone.armDisarm(True) 
    

    locpick = random.randint(0,5)
    target_x = target_locations[locpick, 0]
    target_y = target_locations[locpick, 1]
    target_z = target_locations[locpick, 2]
    print(f"target location randomized: {target_x}, {target_y}, {target_z}")
    heading = Quaternionr(0, 0, 0, 0)
    position = Vector3r(target_x, target_y, target_z)
    pose = Pose(position, heading)
    drone.simSetObjectPose("character_2", pose, True)

    print("target new position: ")
    print(drone.simGetObjectPose("character_2").position)
    print("\n")

    #print("System update: Drone has successfully been reset")
    drone.moveByVelocityAsync(0, 0, 0, 20).join()
    #time.sleep(3)
    num+=1
        
    
    