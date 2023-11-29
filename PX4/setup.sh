#!/bin/bash

# Install QGroundControl dependencies:
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y

# Clone PX4
echo -e "\e[1;34mCloning PX4 Autopilot repository...\e[0m"
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install PX4 Dependencies
echo -e "\e[1;34mInstalling PX4 dependencies...\e[0m"
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

exit

# Compile PX4 without running
echo -e "\e[1;34mCompiling PX4... This can take up to an hour.\e[0m"
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default none_iris
cd ..

# Output message
echo -e "\e[1;32mPX4 Successfully Installed.\e[0m"