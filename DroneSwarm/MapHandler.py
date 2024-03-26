import airsim
import websockets
import asyncio
import numpy as np
import math
from HelperFunctions import clusterHelper
import os
import cv2
from Constants import configDrones
import rospy
import time
import json
# import Constants
import Constants.ros as ros
from threading import Timer # Use for interval checks with minimal code
from threading import Thread # USe for important code running constantly
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import updateMap
from airsim_ros_pkgs.srv import getDroneData, getDroneDataResponse
import threading
god = threading.Lock()

DETECTION_OUTPUT_SERIES = configDrones.DETECTION_OUTPUT_SERIES
ZOOM_FACTOR = configDrones.ZOOM_FACTOR
MAX_PIX_COUNT = configDrones.MAX_PIX_COUNT
SPIRAL_LOCATION_1 = [configDrones.SPIRAL_LOCATION_1[1], configDrones.SPIRAL_LOCATION_1[0]]


# Environmental Variables
# ros: topics
MAP_HANDLER_TOPIC = ros.MAP_HANDLER_TOPIC
FINAL_TARGET_POSITION = ros.FINAL_TARGET_POSITION
NEW_GPS_PREDICTION = ros.NEW_GPS_PREDICTION
UPDATE_DRONE_POSITION =  ros.UPDATE_DRONE_POSITION

PREVIOUS_GPS_POSITIONS = []
PREVIOUS_OVERSEER_GPS_POSITIONS = None
BATCH_DETECTIONS = []
BATCH_TIME = None
START_TIME = None
WOLF_COUNT = None

#Minimap!

# WebSocket server URL
WEBSOCKET_SERVER_URL = "ws://172.23.0.1:8765"


async def register_drone(drone_name, websocket):
    """Send a drone registration message to the server."""
    drone_registration_message = {
        "client_type": "drone_swarm",  # Identifies this client as part of the drone swarm
        "message_type": "CreateDrone",
        "drone_name": drone_name
    }
    await websocket.send(json.dumps(drone_registration_message))
    print(f"Registered drone: {drone_name}")

async def send_auth(websocket):
    """Send a drone registration message to the server."""
    drone_auth_message = {
        "client_type": "drone_swarm",  # Identifies this client as part of the drone swarm
        "message_type": "authentication"
    }
    await websocket.send(json.dumps(drone_auth_message))
    print(f"Authenticated")

async def update_drone(drone_name, latitude, longitude, altitude, velocity, websocket):
    """Send a drone registration message to the server."""
    drone_registration_message = {
        "client_type": "drone_swarm",  # Identifies this client as part of the drone swarm
        "message_type": "UpdateDrone",
        "drone_name": drone_name,
        "latitude": latitude,
        "longitude": longitude,
        "altitude": altitude,
        "velocity": velocity,
    }
    await websocket.send(json.dumps(drone_registration_message))
    print(f"Updated drone: {drone_name}")


async def connect_and_register_drones(drone_names):
    """Connect to the WebSocket server and register each drone."""
    try:
        async with websockets.connect(WEBSOCKET_SERVER_URL) as websocket:
            send_auth(websocket)
            for drone_name in drone_names:
                await register_drone(drone_name, websocket)
            # Keep the connection open to listen for server messages or send updates
            await websocket.wait_closed()
    except Exception as e:
        print(f"Connection failed: {e}")

def start_websocket_client(drone_names):
    """Starts the WebSocket client in an asyncio event loop."""
    asyncio.new_event_loop().run_until_complete(connect_and_register_drones(drone_names))



def startMapHandler(wolfCount):
    print("Starting map handler")

    global BATCH_TIME, START_TIME, WOLF_COUNT
    WOLF_COUNT = wolfCount
    BATCH_TIME = time.time()
    START_TIME = time.time()

    # Assuming globalVarSetup is a function you have defined to setup global variables
    globalVarSetup(wolfCount)

    # Simulating drone names based on the number of drones (wolves)
    drone_names = [f"Drone_{i + 1}" for i in range(wolfCount)]

    # Start the WebSocket client in a separate thread to avoid blocking ROS
    threading.Thread(target=start_websocket_client, args=(drone_names,)).start()

    # Create Node for "ProximityOverseer" (Only one should exist)
    nodeName = "MapHandler"
    rospy.init_node(nodeName, anonymous=True)

    # Subscribe to map handler topic
    print(f"Subscribing to {MAP_HANDLER_TOPIC}")
    mapHandlerSub()  # Assuming this is a function you have defined to subscribe to ROS topics


def globalVarSetup(droneCount):
    global PREVIOUS_GPS_POSITIONS
    PREVIOUS_GPS_POSITIONS = [None]*droneCount


# Connects subcriber listen to MapHandler
def mapHandlerSub():
    rospy.Subscriber(MAP_HANDLER_TOPIC, updateMap, updateResponseQ, ())
    # rospy.Subscriber(MAP_HANDLER_TOPIC, updateMap, updateMapHandler, ())
    rospy.spin()

def updateResponseQ(data, args):
    updateMapHandler(data)
    print("Update Response Q")
    print(data)

    # Extracting wolfPosition
    wolf_position = data.get('wolfPosition', {})

    # Printing wolfPosition variables independently
    latitude = wolf_position.get('latitude', 0)
    longitude = wolf_position.get('longitude', 0)
    altitude = wolf_position.get('altitude', 0)

    print(f"Wolf Position Latitude: {latitude}")
    print(f"Wolf Position Longitude: {longitude}")
    print(f"Wolf Position Altitude: {altitude}")

def updateMapHandler(data):
    # Gets the update type from the data
    updateMapType = data.updateType

    # Get the tuples
    wolfPositionTuple = GPSToTuple(data.wolfPosition)
    targetPositionTuple = GPSToTuple(data.targetPosition)


        imageNumber = data.imageNumber
        vehicleName = data.wolfNumber
        isOverseer = data.isOverseer

        previousGPSPostion = PREVIOUS_OVERSEER_GPS_POSITIONS
        if (not isOverseer):
            previousGPSPostion = PREVIOUS_GPS_POSITIONS[vehicleName]

        # Calls drone and gps update functions
        handleDronePosition(lastWolfPos=previousGPSPostion, wolfPos=wolfPositionTuple, detectMap=detectMap, vehicleName=vehicleName, isOverseer=isOverseer)
        handleGPSPrediction(wolfPos=wolfPositionTuple, predPos=predictedPosition, imageNumber=imageNumber, vehicleName=vehicleName, detectMap=detectMap, isOverseer=isOverseer)

        # Updates previous gps into array
        if (isOverseer):
            PREVIOUS_OVERSEER_GPS_POSITIONS = wolfPositionTuple
        else:
            PREVIOUS_GPS_POSITIONS[vehicleName] = wolfPositionTuple

    # Add drone position
    elif (updateMapType == UPDATE_DRONE_POSITION):
        # Gets necessary variables
        wolfPositionTuple = GPSToTuple(data.wolfPosition)
        vehicleName = data.wolfNumber
        isOverseer = data.isOverseer

        previousGPSPostion = PREVIOUS_OVERSEER_GPS_POSITIONS
        if (not isOverseer):
            previousGPSPostion = PREVIOUS_GPS_POSITIONS[vehicleName]

        # Calls drone and gps update functions
        handleDronePosition(lastWolfPos=previousGPSPostion, wolfPos=wolfPositionTuple, detectMap=detectMap, vehicleName=vehicleName, isOverseer=isOverseer)

        # Updates previous gps into array
        if (isOverseer):
            PREVIOUS_OVERSEER_GPS_POSITIONS = wolfPositionTuple
        else:
            PREVIOUS_GPS_POSITIONS[vehicleName] = wolfPositionTuple

# Converts tuples
def GPSToTuple(GPS):
    return [GPS.latitude, GPS.longitude]

def handleTargetPosition(finalTargetPosition, detectMap):
    # path = calcPath()
    positionMapCenter = SPIRAL_LOCATION_1
    pixCount = MAX_PIX_COUNT
    FUDGE_FACTOR = ZOOM_FACTOR

    # calculate lat and lon offset
    LAT_OFFSET = (pixCount/2) - positionMapCenter[0]*FUDGE_FACTOR
    LON_OFFSET = (pixCount/2) - positionMapCenter[1]*FUDGE_FACTOR

    # if os.path.exists(path):
    #     detectMap = cv2.imread(path)
    # else:
    #     print("Remove image handleTargetPosition-------------------------------------------------------------------------------------------------------------------");
    #     detectionMap = np.zeros((pixCount, pixCount, 3))
    #     cv2.imwrite(path, detectionMap)
    #     detectMap = cv2.imread(path)

    cv2.circle(detectMap, (int(finalTargetPosition[0]*FUDGE_FACTOR + LAT_OFFSET), abs(int(finalTargetPosition[1]*FUDGE_FACTOR + LON_OFFSET))), 5, (240, 32, 160), 2) # draw ground truth postion
    # cv2.imwrite(path, detectMap)


def handleGPSPrediction(wolfPos, predPos, imageNumber, vehicleName, detectMap, isOverseer):
    # path = calcPath()
    positionMapCenter = SPIRAL_LOCATION_1
    pixCount = MAX_PIX_COUNT
    FUDGE_FACTOR = ZOOM_FACTOR

    # calculate lat and lon offset
    LAT_OFFSET = (pixCount/2) - positionMapCenter[0]*FUDGE_FACTOR
    LON_OFFSET = (pixCount/2) - positionMapCenter[1]*FUDGE_FACTOR

    # if os.path.exists(path):
    #     print("Path exists, doing read")
    #     detectMap = cv2.imread(path)
    # else:
    #     print("Remove image handleGPSPrediction-------------------------------------------------------------------------------------------------------------------");
    #     detectionMap = np.zeros((pixCount, pixCount, 3))
    #     cv2.imwrite(path, detectionMap)
    #     detectMap = cv2.imread(path)
    predictedColor = (15, 155, 228)
    lineWeight = 3
    textPred = 'O_' + str(imageNumber)
    textPredColor = (255, 255, 255)
    if not isOverseer:
        predictedColor = (255, 0, 0)
        lineWeight = 2
        textPred = str(imageNumber) + 'w' + str(vehicleName)
        textPredColor = (0, 0, 255)
        if (imageNumber < 0): # fail prediction
            predictedColor = (0,255,255)

    cv2.line(detectMap, (int(wolfPos[0]*FUDGE_FACTOR + LAT_OFFSET), abs(int(wolfPos[1]*FUDGE_FACTOR + LON_OFFSET))), (int(predPos[0]*FUDGE_FACTOR + LAT_OFFSET), abs(int(predPos[1]*FUDGE_FACTOR + LON_OFFSET))), (255, 255, 0), 2) # draw line between wolf and prediction
    cv2.circle(detectMap, (int(wolfPos[0]*FUDGE_FACTOR + LAT_OFFSET), abs(int(wolfPos[1]*FUDGE_FACTOR + LON_OFFSET))), 2, (0, 0, 255), 2) # draw vehicle postion
    cv2.circle(detectMap, (int(predPos[0]*FUDGE_FACTOR + LAT_OFFSET), abs(int(predPos[1]*FUDGE_FACTOR + LON_OFFSET))), lineWeight, predictedColor, lineWeight) # draw predicted gps postion for wolf
    cv2.putText(detectMap, textPred, (int(wolfPos[0]*FUDGE_FACTOR + LAT_OFFSET) + 1, abs(int(wolfPos[1]*FUDGE_FACTOR + LON_OFFSET)) + 1), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1)
    # cv2.imwrite(path, detectMap)

def handleDronePosition(lastWolfPos, wolfPos, detectMap, vehicleName, isOverseer):
    # path = calcPath()
    positionMapCenter = SPIRAL_LOCATION_1
    pixCount = MAX_PIX_COUNT
    FUDGE_FACTOR = ZOOM_FACTOR
    global WOLF_COUNT





    # calculate lat and lon offset
    LAT_OFFSET = (pixCount/2) - positionMapCenter[0]*FUDGE_FACTOR
    LON_OFFSET = (pixCount/2) - positionMapCenter[1]*FUDGE_FACTOR

    # if os.path.exists(path):
    #     detectMap = cv2.imread(path)
    # else:
    #     detectionMap = np.zeros((pixCount, pixCount, 3))
    #     cv2.imwrite(path, detectionMap)
    #     detectMap = cv2.imread(path)

    # if type(detectMap) == None or detectMap.size == 0:
    #     print("READ ERRORRRRRRRRRRRRRRR")
    #     return

    droneColor = (255, 255, 255)
    lineWeight = 2
    if not isOverseer:
        droneColor = (255*( 1 - vehicleName/WOLF_COUNT), 140, 255*(vehicleName/WOLF_COUNT))
        lineWeight = 1

    print("New Wolf Pos:")
    print(vehicleName)
    print(wolfPos)

def calcPath():
    return '/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/detectionMaps' + '/' + 'detections' + '_' + DETECTION_OUTPUT_SERIES + '_' + '.jpeg'