import gymnasium
import CustomEnv
import time
import os

from stable_baselines3 import DQN
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import CheckpointCallback, StopTrainingOnNoModelImprovement, ProgressBarCallback
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import torch as th
from torch import nn
# from torchvision.models import resnet18, ResNet18_Weights
import torchvision.models as models
import torch.nn.functional as F

from stable_baselines3.common.env_checker import check_env
# try learning rate schedule
from typing import Callable, Dict

logdir = "logs"
models_dir = "models/DQN"

if not os.path.exists(models_dir): #create directories if they dont already exist
    os.makedirs(models_dir)

if not os.path.exists(logdir):
    os.makedirs(logdir)

class CustomCombinedExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: gymnasium.spaces.Dict):
        # super().__init__(observation_space, features_dim=512)  # Adjust features_dim as needed
        super(CustomCombinedExtractor, self).__init__(observation_space, features_dim=1)

        # Define ResNet-18 backbone
        # ResNet18's original conv1 parameters: 64 output channels, kernel size of 7, stride of 2, padding of 3
        n_input_channels = observation_space["image"].shape[0]
        self.resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        self.resnet.conv1 = nn.Conv2d(n_input_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)  # Adjust first convolutional layer
        
        # # Remove the fully connected layer and average pooling layer
        # self.resnet = nn.Sequential(*list(self.resnet.children())[:-2])

        # Number of input features for the final linear layer (should remain the same as before modification)
        num_ftrs = self.resnet.fc.in_features
        
        # Define the number of output features for your task, e.g., 10 for a 10-class classification
        num_output_features = 9 # change this to 9 and debug
        
        # Replace the last fully connected layer with a custom linear layer
        self.resnet.fc = nn.Linear(num_ftrs, num_output_features)

        # Define additional feature extractors for non-image features
        self.additional_extractors = nn.ModuleDict({
            "velocity": nn.Sequential(nn.Linear(observation_space["velocity"].shape[0], 16), nn.ReLU()),
            "relative_distance": nn.Sequential(nn.Linear(observation_space["relative_distance"].shape[0], 16), nn.ReLU()),
            "prev_relative_distance": nn.Sequential(nn.Linear(observation_space["prev_relative_distance"].shape[0], 16), nn.ReLU()),
            "action_history": nn.Sequential(nn.Linear(observation_space["action_history"].shape[0], 16), nn.ReLU()),
            "altimeter": nn.Sequential(nn.Linear(observation_space["altimeter"].shape[0], 16), nn.ReLU())
        })

        # Calculate total feature dimension
        total_concat_size = 0
        for key in observation_space.spaces:
            total_concat_size += 16  # Assuming 16 features for each non-image feature
        
        # Update the total feature dimension
        self._features_dim += total_concat_size - 8 #there's an 8 extra for some reason so we're subtracting?

    def forward(self, observations: Dict[str, th.Tensor]) -> th.Tensor:
        # Process image observations with ResNet-18 backbone
        image = observations["image"] / 255.  # Normalize image
        image_features = self.resnet(image)
        image_features = th.flatten(image_features, start_dim=1)
        
    
        encoded_tensor_list = []
        encoded_tensor_list.append(image_features)

        # Process non-image observations
        for key, extractor in self.additional_extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
    

        combined_features = th.cat(encoded_tensor_list, dim=1)


        return combined_features
    
policy_kwargs = dict(
            features_extractor_class=CustomCombinedExtractor,
        )

# check_env(gymnasium.make("airsim-drone-dynamic-v0"))
# from CustomEnv.envs.gym_env_dynamic import drone_env_dynamic
'''For dynamic environment'''
# env = DummyVecEnv([lambda: Monitor(gymnasium.make("airsim-drone-dynamic-v0"))])
'''Fpr static environment'''
env = DummyVecEnv([lambda: Monitor(gymnasium.make("airsim-drone-v0"))])
env = VecTransposeImage(env)


model = DQN(
"MultiInputPolicy",
env,
learning_rate=0.01,
verbose=1,
batch_size=128,
train_freq=4,
target_update_interval=4_000,
learning_starts=2500 ,
policy_kwargs=policy_kwargs,
buffer_size=50_000,
max_grad_norm=10,
exploration_fraction=0.8,
exploration_final_eps=0.03,
# device="cuda",
tensorboard_log=logdir
)

callbacks = []

checkpoint_callback = CheckpointCallback(
    save_freq=7_000,
    save_path="./CheckPoints/DQN/CheckPoint/",
    name_prefix="rl_model",
    save_replay_buffer=False,
    # save_replay_buffer=True,
    save_vecnormalize=True,
)

stop_train_callback = StopTrainingOnNoModelImprovement(
    max_no_improvement_evals=20, 
    min_evals=80, 
    verbose=1
)

eval_callback = EvalCallback(
    env,
    callback_on_new_best=None,
    n_eval_episodes=5,
    callback_after_eval=stop_train_callback,
    best_model_save_path="./CheckPoints/DQN/BestModel/",
    log_path="./CheckPoints/DQN/BestModel/",
    eval_freq=500,
    # eval_freq=400,
)

# Create a progress bar callback to estimate time left
progress_bar_callback = ProgressBarCallback()
callbacks.append(progress_bar_callback)

callbacks.append(eval_callback)
callbacks.append(checkpoint_callback)

kwargs = {}
kwargs["callback"] = callbacks

TIMESTEPS = 80_000

model.learn(
    total_timesteps=TIMESTEPS,
    # reset_num_timesteps=False,
    reset_num_timesteps=True,
    tb_log_name="DQN",
    **kwargs
    )
    
model.save(f"{models_dir}/{TIMESTEPS}")
