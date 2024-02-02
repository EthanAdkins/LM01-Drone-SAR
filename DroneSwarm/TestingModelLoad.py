#import tensorflow as tf
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
    def get_available_devices():
        local_device_protos = device_lib.list_local_devices()
        return [x.name for x in local_device_protos]
    # print(tf.test.is_built_with_cuda())
    print(get_available_devices())
    
    model_path = "/home/testuser/AirSim/PythonClient/multirotor/LM01-Drone-SAR/DroneSwarm/final_save"

    env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airgym:airsim-drone-sample-v1",
                ip_address="172.28.160.1",
                step_length=0.25,
                image_shape=(19,),
            )
        )
    ])
    env.render_mode = "rgb_array"    
    try:
        print("Loading DQN Model")
        rlModel = DQN.load(model_path)
        print("DQN Model Loaded")
    except Exception as error:
        print("DQN Model FAILED: ", error)

    obs = env.reset()
    while True:
        action, _states = rlModel.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()
