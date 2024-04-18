# Set Execution Policy
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process

# Initialize Anaconda Environment
& 'C:\Users\Admin\anaconda3\shell\condabin\conda-hook.ps1'
conda activate 'C:\Users\Admin\anaconda3'

# Activate ml_env Environment
conda activate ml_env

# Add ROS setup to .bashrc (Assuming usage of bash within WSL or compatible terminal)
$profilePath = [System.Environment]::GetFolderPath('UserProfile') + '\.bashrc'
Add-Content -Path $profilePath -Value 'source /opt/ros/melodic/setup.bash'

# Attempt to source .bashrc (requires bash to be accessible)
& bash -c 'source ~/.bashrc'

# Build Docker Image
docker build -t airsim-ros-pytorch-newest -f Dockerfile-ROS-Pytorch-Newest .

# Run Docker Container
docker run --rm -it --net="host" -v //c/SD2/Colosseum:/home/testuser/AirSim airsim-ros-pytorch-newest:latest bash -c "cd AirSim/ros && export ROS_HOSTNAME=172.17.0.1 && source devel/setup.bash && cd .. && cd PythonClient/multirotor/LM01-Drone-SAR/DroneSwarm && export ROS_HOSTNAME=172.17.0.1 && python3 MissionControl.py"

#Read-Host -Prompt "Press any key to continue"