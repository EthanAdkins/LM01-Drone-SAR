import gymnasium
gymnasium.register

gymnasium.register(
    id="airsim-drone-sample-v0", entry_point="airgym.envs:AirSimDroneEnv"
)

gymnasium.register(
    id="airsim-drone-sample-v1", entry_point="airgym.envs:AirSimDroneEnvV1"
)

gymnasium.register(
    id="airsim-car-tracking-v1", entry_point="airgym.envs:DroneCarTrackingEnv"
)

gymnasium.register(
    id="airsim-drone-traversal-demo-v0", entry_point="airgym.envs:DroneTraversalDemo"

)

gymnasium.register(
    id="airsim-car-tracking-demo-v0", entry_point="airgym.envs:DroneCarTrackingDemo"
)