import gymnasium as gym


gym.register(
    id="airsim-drone-v0",
    entry_point="CustomEnv.envs:drone_env"
)

gym.register(
    id="airsim-drone-dynamic-v0",
    entry_point="CustomEnv.envs:drone_env_dynamic"
)

gym.register(
    id="airsim-drone-continuous-v0",
    entry_point="CustomEnv.envs:drone_env_continuous"
)

gym.register(
    id="airsim-drone-continuous-dynamic-v0",
    entry_point="CustomEnv.envs:drone_env_continuous_dynamic"
)