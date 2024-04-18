<!-- CONDA ENV -->
## Managing CONDA environments

To ensure consistent testing, this is where we will put the conda environments we are using. It's best to use the Anaconda Navigator because it has a GUI!

### Importing Environments
1. Make a conda environment by either using the command line or Anaconda Navigator
2. Grab the requirements.txt file that's in this directory
3. Install the following dependencies first
 ```sh
   pip install numpy msgpack-rpc-python
   ```
4. Then run this command to install the rest of the packages that are in the requirements.txt
 ```sh
   pip install -r requirements.txt
   ```

### Exporting Environments
1. Using either the terminal or the Anaconda Navigator, go to the conda environment you want to get a list of all the packages from
2. Run the command
 ```sh
   pip freeze > requirements.txt
   ```
3. A text file containing a list of all the packages being used in the environment should have been create

If you want to use git bash as your terminal to run conda commands, follow this guide: https://discuss.codecademy.com/t/setting-up-conda-in-git-bash/534473


<!-- Settings Config for AirSim -->
## Settings Config for AirSim
Several settings might be used when testing with AirSim, so we will also keep those settings.json here (for reinforcement learning at least)

### Customizing settings.json
1. To locate your settings.json, go to "C:\Users\xxxx\Documents\AirSim"
2. Change the contents of the json file by copy-pasting.
