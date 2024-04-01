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

import ServiceRequestors.overseerGetWolfData as overseerGetWolfData
import ServiceRequestors.overseerGetOverseerData as overseerGetOverseerData
import ServiceRequestors.overseerGetWolfData as overseerGetWolfData

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
OVERSEER_DATA_TOPIC = ros.OVERSEER_DATA_TOPIC
OVERSEER_COMMUNICATION_TOPIC = ros.OVERSEER_COMMUNICATION_TOPIC

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

async def connect_and_listen(uri):
    global websocket_connection
    while True:
        try:
            websocket_connection = await websockets.connect(uri)
            print("Connected to WebSocket server at:", uri)
            auth_message = json.dumps({"client_type": "swarm", "message_type": "authentication"})
            await websocket_connection.send(auth_message)
            print("Authentication message sent.")
            # Listen for and handle incoming messages
            async for message in websocket_connection:
                print(f"Received message: {message}")
                # Insert handling of incoming message here
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed, attempting to reconnect...")
            await asyncio.sleep(5)
        except Exception as e:
            print(f"WebSocket error: {e}")
            await asyncio.sleep(5)



def start_websocket_connection(uri):
    """Schedules the WebSocket connection in the event loop."""

    # Initialize the event loop in a separate thread
    thread = Thread(target=start_asyncio_loop)
    thread.daemon = True
    thread.start()

    asyncio.run_coroutine_threadsafe(connect_and_listen(uri), asyncio.get_event_loop())


def start_asyncio_loop():
    """Starts the asyncio event loop in a background thread."""
    loop = asyncio.new_event_loop()  # Create a new event loop
    asyncio.set_event_loop(loop)  # Set the new event loop as the current one
    loop.run_forever()

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
    print("Starting map handler...")
    # Schedule WebSocket connection
    start_websocket_connection(WEBSOCKET_SERVER_URL)

    print("Map socket fully started")

    global BATCH_TIME, START_TIME, WOLF_COUNT
    WOLF_COUNT = wolfCount
    BATCH_TIME = time.time()
    START_TIME = time.time()
    # Sets up empty arrays
    globalVarSetup(wolfCount)

    nodeName = "MapHandler"
    rospy.init_node(nodeName, anonymous=True)

    subTopics()

def subTopics():
    # Subscribe to map handler topic

    #rospy.Subscriber(OVERSEER_COMMUNICATION_TOPIC, String, handleOverseerCommunication, ())
    rospy.Subscriber(OVERSEER_DATA_TOPIC, droneData, updateOverseerData, ())
    rospy.spin()


def updateOverseerData(data, args):
    drone_name = data.droneName
    cluster_name = data.cluster

    wolf_drones_data = overseerGetWolfData.getWolfDataOfCluster(cluster_name)
    wolf_names = [drone.droneName for drone in wolf_drones_data]

    # Construct the "UpdateClusters" message
    update_clusters_message = json.dumps({
        "message_type": "UpdateClusters",
        "cluster_name": cluster_name,
        "drone_names": wolf_names
    })

    print("Updating cluster1111")
    # Send the message to the WebSocket server
    send_message(update_clusters_message)
    print("Updating cluster")


def globalVarSetup(droneCount):
    global PREVIOUS_GPS_POSITIONS
    PREVIOUS_GPS_POSITIONS = [None] * droneCount
