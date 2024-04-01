import airsim
import Constants.configDrones as configDrones
import ImageProcessing.getInfo as getInfo
import time
import math
import os
import cv2
from ImageProcessing import getInfo
# Connect to the AirSim simulator
LOCAL_IP = configDrones.LOCAL_IP
droneName = "0"
client = airsim.MultirotorClient(LOCAL_IP)
client.confirmConnection()
client.enableApiControl(True, droneName)
client.armDisarm(True, droneName)
client.takeoffAsync(vehicle_name=droneName).join()
desiredHeight = -40
# Function to convert degrees to radians
def deg_to_rad(deg):
    return deg * math.pi / 180.0

# Function to calculate orbit position
def calculate_orbit_position(center, radius, angle):
    x = center[0] + radius * math.cos(deg_to_rad(angle))
    y = center[1] + radius * math.sin(deg_to_rad(angle))
    return (x, y, center[2])  # Maintaining the altitude of the center point

# Create a directory to store images if it doesn't exist
image_directory = "images"
if not os.path.exists(image_directory):
    os.makedirs(image_directory)

# Location to orbit around (adjust as needed)
orbit_center = (16.0325818 - 10, -164.50358846 - 4, desiredHeight)
orbit_radius = 10  # Adjust the orbit radius as needed

# Number of steps for one complete orbit
num_steps = 36  # Change this to adjust the granularity of the orbit (360/num_steps gives the angle increment)

# Inside the loop for taking pictures
for i in range(num_steps):
    angle = i * (360 / num_steps)
    position = calculate_orbit_position(orbit_center, orbit_radius, angle)
    client.moveToPositionAsync(*position, velocity=8, vehicle_name=droneName).join()
    # Take a picture
    responses = client.simGetImages([
        airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)
    ], vehicle_name = droneName)
    
    for j, response in enumerate(responses):
        height, width, sceneRGB2 = getInfo.getHeightWidthArr(responses, 0)
        cv2.imwrite(os.path.join(image_directory, f"BrianAndAnimal_{i + 36}.png"), sceneRGB2)  # Save the image
       
    # Get GPS data
    time.sleep(3)  # Adjust this sleep time as needed
    _, lat, lon = getInfo.getDroneGPS("0", client)
    
    # Print GPS coordinates
    print("Orbit Position: {}, Angle: {}".format(position, angle))
    print("Latitude: {}".format(lat))
    print("Longitude: {}\n".format(lon))

print("Orbit completed.")
