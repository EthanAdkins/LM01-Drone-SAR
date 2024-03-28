import airsim
import numpy as np
import airsim
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
from threading import Timer  # Use for interval checks with minimal code
from threading import Thread  # USe for important code running constantly
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import updateMap
from airsim_ros_pkgs.srv import getDroneData, getDroneDataResponse
import threading

import asyncio
import websockets
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
UPDATE_DRONE_POSITION = ros.UPDATE_DRONE_POSITION

PREVIOUS_GPS_POSITIONS = []
PREVIOUS_OVERSEER_GPS_POSITIONS = None
BATCH_DETECTIONS = []
BATCH_TIME = None
START_TIME = None
WOLF_COUNT = None


# Global variable to store the WebSocket connection
websocket_connection = None
WEBSOCKET_SERVER_URL = "ws://172.23.0.1:8765"

# Event loop for asynchronous tasks
loop = asyncio.get_event_loop()

async def connect_to_websocket_server(uri):
    global websocket_connection
    websocket_connection = await websockets.connect(uri)
    print("Connected to WebSocket server at:", uri)
    # Construct and send the authentication message
    auth_message = json.dumps({"client_type": "swarm", "message_type": "authentication"})
    await websocket_connection.send(auth_message)
    print("Authentication message sent.")

def start_websocket_connection(uri=WEBSOCKET_SERVER_URL):  # Ensure URI matches your server
    # Run the connection in the event loop
    loop.run_until_complete(connect_to_websocket_server(uri))

async def send_message_async(message):
    global websocket_connection
    if websocket_connection:
        await websocket_connection.send(message)
    else:
        print("WebSocket connection is not established.")

def send_message(message):
    # Wrapper to send a message through the WebSocket from synchronous code
    asyncio.run_coroutine_threadsafe(send_message_async(message), loop)

# Main Process Start ----------------------------------------------
def startMapHandler(wolfCount):
    print("Starting map handler")
    # Initiate WebSocket connection
    start_websocket_connection()

    global BATCH_TIME, START_TIME, WOLF_COUNT
    WOLF_COUNT = wolfCount
    BATCH_TIME = time.time()
    START_TIME = time.time()
    # Sets up empty arrays
    globalVarSetup(wolfCount)

    nodeName = "MapHandler"
    rospy.init_node(nodeName, anonymous=True)


def globalVarSetup(droneCount):
    global PREVIOUS_GPS_POSITIONS
    PREVIOUS_GPS_POSITIONS = [None] * droneCount
