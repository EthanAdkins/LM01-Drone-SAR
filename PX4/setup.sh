#!/bin/bash
# /mnt/c/Users/Admin/Documents/Colosseum/PythonClient/multirotor/test/PX4
sudo apt-get update

# Clone PX4
echo -e "\e[1;34mCloning PX4 Autopilot repository...\e[0m"
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install PX4 Dependencies
echo -e "\e[1;34mInstalling PX4 dependencies...\e[0m"
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# Compile PX4 without running
echo -e "\e[1;34mCompiling PX4... This can take up to an hour.\e[0m"
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default none_iris
cd ..

# Output message
echo -e "\e[1;32mPX4 Successfully Installed.\e[0m"