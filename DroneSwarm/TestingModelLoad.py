#import tensorflow as tf
import airsim
import threading
from tensorflow.python.client import device_lib
from stable_baselines3 import DQN
import gym
import airgym
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.monitor import Monitor
# from stable_baselines3 import DQN
# import gymnasium as gym
# from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
# from stable_baselines3.common.monitor import Monitor

if __name__ == '__main__':
    
    drone = airsim.MultirotorClient(ip="172.28.160.1")
    drone.reset()


    drone.enableApiControl(True, "0")
    drone.armDisarm(True, "0")
    drone.enableApiControl(True, "1")
    drone.armDisarm(True, "1")

    drone.takeoffAsync(vehicle_name="0").join()
    drone.takeoffAsync(vehicle_name="1").join()

    def get_available_devices():
        local_device_protos = device_lib.list_local_devices()
        return [x.name for x in local_device_protos]
    
    def interpret_action(action):
        rotate = 0
        if action == 0:
            quad_offset = (0.25, 0, 0)
        elif action == 1:
            quad_offset = (0, 0.25, 0)
        elif action == 2:
            quad_offset = (0, 0, 0.25)
        elif action == 3:
            quad_offset = (-0.25, 0, 0)
        elif action == 4:
            quad_offset = (0, -0.25, 0)
        elif action == 5:
            quad_offset = (0, 0, -0.25)
        elif action == 6:
            rotate = 1
            quad_offset = -30
        elif action == 7:
            rotate = 1
            quad_offset = 30
        else:
            quad_offset = (0, 0, 0)

        return quad_offset, rotate
    
    def move_drone(drone, action, vehicle_name):
        quad_vel = drone.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated.linear_velocity
        quad_offset, drone_rotate = interpret_action(action)
        
        if drone_rotate == 0:
            drone.moveByVelocityAsync(
                quad_vel.x_val + quad_offset[0],
                quad_vel.y_val + quad_offset[1],
                quad_vel.z_val + quad_offset[2],
                .5,
                vehicle_name=vehicle_name
            ).join()
        else:
            drone.rotateByYawRateAsync(quad_offset, .5, vehicle_name=vehicle_name)
    
    # print(tf.test.is_built_with_cuda())
    print(get_available_devices())
    
    model_path = "/home/testuser/AirSim/PythonClient/multirotor/LM01-Drone-SAR/DroneSwarm/best_model"

    env0 = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airgym:airsim-drone-sample-v1",
                ip_address="172.28.160.1",
                step_length=0.25,
                image_shape=(19,),
                drone_name="0"
            )
        )
    ])

    env1 = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airgym:airsim-drone-sample-v1",
                ip_address="172.28.160.1",
                step_length=0.25,
                image_shape=(19,),
                drone_name="1"
            )
        )
    ])

    try:
        print("Loading DQN Model")
        rlModel = DQN.load(model_path)
        print("DQN Model Loaded")
    except Exception as error:
        print("DQN Model FAILED: ", error)

    obs0 = env0.reset()
    obs1 = env1.reset()

    while True:
        action0, _states0 = rlModel.predict(obs0)
        action1, _states1 = rlModel.predict(obs1)

        obs0, rewards0, dones0, info0 = env0.step(action0)
        obs1, rewards1, dones1, info1 = env1.step(action1)

        # Move drone 0
        move_drone(drone, action0, "0")

        # Move drone 1
        move_drone(drone, action1, "1")


        #env.render()

