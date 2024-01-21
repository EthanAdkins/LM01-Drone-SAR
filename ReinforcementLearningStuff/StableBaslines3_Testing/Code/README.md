<!-- SB3 RL Code -->
## SB3 RL Code 

Most of the stuff here I (Andrei) have not touched yet, but this directory contains a good amount of code which could be a good start for our RL training for drones :D

### P.S. Some files are still outdated!
Some of the files here use deprecated functions. The files that have been updated to be compatible with libraries are:
- SB3_DQN.py
- SB3_ModelTesting.py
- gym_env_dynamic.py (this is a custom env, do not run directly)
- gym_env.py (this is a custom env, do not run directly)

### Prerequisites

Before running the code, make sure that:
* You have the settings.json configured similar to the one in TestingEnv
* Your conda environment is compatible with the codebase (check TestingEnv)

### How to run the code (training the model)
1. Get your AirSim running first and click play to spawn a drone!
2. In your terminal, make sure you are in the same directory where the file is. To get the tensorboard running, run:
   ```sh
   python -m tensorboard.main --logdir=logs
   ```
3. In your terminal, make sure you are in the same directory where the file is. If you want to run a DQN training, do
   ```sh
   python ./SB3_DQN.py
   ```

### How to run the code (testing trained model)
1. Get your AirSim running first and click play to spawn a drone!
2. In SB3_ModelTesting.py, make sure the right environment is selected for testing (is it for continuous or discrete? static or dynamic?)
3. In SB3_ModelTesting.py, make sure that the filepath for the model you want to train is correct (it's hard-coded right now so double check!)
4. To test your trained model, run the following in your terminal
   ```sh
   python ./SB3_ModelTesting.py
   ```
   Once the total number of runs finishes, it should return a percentage of successful runs.
