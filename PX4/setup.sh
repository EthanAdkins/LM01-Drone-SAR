#!/bin/bash
# /mnt/c/Users/Admin/Documents/Colosseum/PythonClient/multirotor/test/PX4
sudo apt-get update

# Clone PX4
mkdir src
cd src
echo -e "\e[1;34mCloning PX4 Autopilot repository...\e[0m"
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install PX4 Dependencies
echo -e "\e[1;34mInstalling PX4 dependencies...\e[0m"
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-simulation

# Output message
echo -e "\e[1;32mPX4 Successfully Installed.\e[0m"