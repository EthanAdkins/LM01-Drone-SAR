import gymnasium
import CustomEnv
import time
import os

import stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import CheckpointCallback, StopTrainingOnNoModelImprovement, ProgressBarCallback
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import torch as th
import torchvision.models as models
from torch import nn
from typing import Callable, Dict

logdir = "logs"
#models_dir = "models/DQN"
models_dir = "models/PPO"

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

        # implement MLP 

        return combined_features
    # def __init__(self, observation_space: gymnasium.spaces.Dict):
    #     # We do not know features-dim here before going over all the items,
    #     # so put something dummy for now. PyTorch requires calling
    #     # nn.Module.__init__ before adding modules
    #     super(CustomCombinedExtractor, self).__init__(observation_space, features_dim=1)

    #     extractors = {}

    #     total_concat_size = 0
    #     # We need to know size of the output of this extractor,
    #     # so go over all the spaces and compute output feature sizes
    #     for key, subspace in observation_space.spaces.items():
    #         #print(key)
    #         if key == "image":
    #             # We assume CxHxW images (channels first)
    #             # Re-ordering will be done by pre-preprocessing or wrapper
                
    #             n_input_channels = subspace.shape[0]
    #             extractors[key] = nn.Sequential(
    #                 nn.Conv2d(n_input_channels, 32, kernel_size=4, stride=4, padding=0),
    #                 nn.ReLU(),
    #                 nn.Conv2d(32, 64, kernel_size=1, stride=2, padding=0),
    #                 nn.ReLU(),
    #                 nn.Conv2d(64, 64, kernel_size=1, stride=1, padding=0),
    #                 nn.ReLU(),
    #                 nn.Flatten(),
    #             )
                
    #             # Compute shape by doing one forward pass
    #             with th.no_grad():
    #                 ex_shape = extractors[key](th.as_tensor(observation_space.sample()[key]).float())
                    
    #             linear = nn.Sequential(nn.Linear(ex_shape.shape[0] * ex_shape.shape[1], 256, nn.ReLU()))  #256 is img features dim
    #             extractors[key] = nn.Sequential(extractors[key], linear)
    #             total_concat_size += 256
            
    #         elif key == "velocity":
    #             extractors[key] = nn.Sequential(
    #                 nn.Linear(subspace.shape[0], 16),
    #                 nn.ReLU()
    #             )
    #             total_concat_size += 16
            
    #         elif key == "relative_distance":
    #             extractors[key] = nn.Sequential(
    #                 nn.Linear(subspace.shape[0], 16),
    #                 nn.ReLU()
    #             )
    #             total_concat_size += 16
            
    #         elif key == "prev_relative_distance":
    #             extractors[key] = nn.Sequential(
    #                 nn.Linear(subspace.shape[0], 16),
    #                 nn.ReLU()
    #             )
    #             total_concat_size += 16
            
    #         elif key == "altimeter":
    #             extractors[key] = nn.Sequential(
    #                 nn.Linear(subspace.shape[0], 16),
    #                 nn.ReLU()
    #             )
    #             total_concat_size += 16
            
    #     self.extractors = nn.ModuleDict(extractors)

    #     # Update the features dim manually
    #     self._features_dim = total_concat_size
    
    # def forward(self, observations) -> th.Tensor:
    #     encoded_tensor_list = []

    #     for key, extractor in self.extractors.items():
    #         encoded_tensor_list.append(extractor(observations[key]))

    #     return th.cat(encoded_tensor_list, dim=1)

policy_kwargs = dict(
            features_extractor_class=CustomCombinedExtractor,
        )

env = DummyVecEnv([lambda: Monitor(gymnasium.make("airsim-drone-continuous-v0"))])
env = VecTransposeImage(env)

# env.action_space

model = PPO(
    "MultiInputPolicy",
    env,
    learning_rate=0.01,
    verbose=1,
    batch_size=256,
    max_grad_norm=0.5,
    clip_range=0.10,
    device="cuda",
    policy_kwargs=policy_kwargs,
    tensorboard_log=logdir,
    
)

#model = PPO.load(r"C:\Users\User\Desktop\ThesisUnReal\CheckPoints\PPO\CheckPoint\rl_model_21000_steps.zip", env=env, tensorboard_log=logdir)

callbacks = []

checkpoint_callback = CheckpointCallback(
  save_freq=7_000,
  save_path="./CheckPoints/PPO/CheckPoint/",
  name_prefix="rl_model",
  save_replay_buffer=True,
  save_vecnormalize=True,
)

stop_train_callback = StopTrainingOnNoModelImprovement(
    max_no_improvement_evals=10, 
    min_evals=150, 
    verbose=1
)

eval_callback = EvalCallback(
    env,
    callback_on_new_best=None,
    n_eval_episodes=5,
    callback_after_eval=stop_train_callback,
    best_model_save_path="./CheckPoints/PPO/BestModel/",
    log_path="./CheckPoints/PPO/BestModel/",
    eval_freq=500,
)


progress_bar_callback = ProgressBarCallback()

callbacks.append(eval_callback)
callbacks.append(checkpoint_callback)
callbacks.append(progress_bar_callback)

kwargs = {}
kwargs["callback"] = callbacks

TIMESTEPS = 100_000

model.learn(
    total_timesteps=TIMESTEPS,
    reset_num_timesteps=True,
    tb_log_name="PPO",
    **kwargs
    )
    
model.save(f"{models_dir}/{TIMESTEPS}")