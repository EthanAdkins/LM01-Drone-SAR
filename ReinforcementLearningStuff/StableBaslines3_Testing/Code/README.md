<!-- SB3 RL Code -->
## SB3 RL Code 

Most of the stuff here I (Andrei) have not touched yet, but this directory contains a good amount of code which could be a good start for our RL training for drones :D

### P.S. Some files are still outdated!
Some of the files here use deprecated functions. The files that have been updated to be compatible with libraries are:
- SB3_DQN.py
- gym_env_dynamic.py (this is a custom env, do not run directly)

### Prerequisites

Before running the code, make sure that:
* You have the settings.json configured similar to the one in TestingEnv
* Your conda environment is compatible with the codebase (check TestingEnv)

### How to run the code
1. Get your AirSim running first and click play to spawn a drone!
2. In your terminal, make sure you are in the same directory where the file is. To get the tensorboard running, run:
   ```sh
   python -m tensorboard.main --logdir=logs
   ```
3. In your terminal, make sure you are in the same directory where the file is. If you want to run a DQN training, do
   ```sh
   python ./SB3_DQN.py
   ```

