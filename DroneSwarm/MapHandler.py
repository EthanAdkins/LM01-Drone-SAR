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
import queue

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
GRID_UPDATED_TOPIC = ros.GRID_UPDATED_TOPIC
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
message_queue = queue.Queue()

async def websocket_client(uri, message_handler):
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print(f"Connected to WebSocket server at: {uri}")
                # Authentication message
                await websocket.send(json.dumps({"client_type": "swarm", "message_type": "authentication"}))
                print("Authentication message sent.")

                async def send_from_queue():
                    while True:
                        message = await asyncio.get_event_loop().run_in_executor(None, message_queue.get)
                        await websocket.send(message)
                        message_queue.task_done()

                sender_task = asyncio.ensure_future(send_from_queue())

                while True:
                    message = await websocket.recv()
                    print(f"Received message: {message}")
                    await message_handler(message)
        except Exception as e:
            print(f"WebSocket error: {e}, attempting to reconnect in 5 seconds...")
            await asyncio.sleep(5)


def send_message(message):
    global message_queue
    # Convert the message to JSON format if it's not already a string
    if not isinstance(message, str):
        message = json.dumps(message)
    message_queue.put_nowait(message)


async def handle_message(message):
    print(f"Processing message: {message}")
    # Process the message here


def start_websocket_client_thread(uri):
    def thread_target():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(websocket_client(uri, handle_message))
        loop.close()

    thread = threading.Thread(target=thread_target, daemon=True)
    thread.start()

def startMapHandler(wolfCount):
    print("Starting map handler...")
    start_websocket_client_thread(WEBSOCKET_SERVER_URL)
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
    print("Subscribing on map handler")

    #rospy.Subscriber(OVERSEER_COMMUNICATION_TOPIC, String, handleOverseerCommunication, ())
    rospy.Subscriber(OVERSEER_DATA_TOPIC, droneData, updateOverseerData, ())
    rospy.Subscriber(GRID_UPDATED_TOPIC, String, handleGridUpdate)
    rospy.spin()

    print("Subscribed on map handler")


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
    # Send the message to the WebSocket server
    send_message(update_clusters_message)

def handleGridUpdate(data):

    # Construct the "UpdateClusters" message
    update_grid_message = json.dumps({
        "message_type": "UpdateGrid",
        "data": data.data
    })

    print("Sending grid!")
    # Send the message to the WebSocket server
    send_message(update_grid_message)


def globalVarSetup(droneCount):
    global PREVIOUS_GPS_POSITIONS
    PREVIOUS_GPS_POSITIONS = [None] * droneCount
