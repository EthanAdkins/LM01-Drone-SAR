import json
import asyncio
from airsim import MultirotorClient
import Messages
import Data
from Data import DroneData
from Messages import UpdateDrone

landscape_offset_x = 15407.818462/100
landscape_offset_y = -103271.472497/100
landscape_offset_z = 25060.063755/100

class DroneMonitor:
    def __init__(self, broadcast_callback):
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.broadcast = broadcast_callback

        #Setup monitoring data
        self.drones = {}

    def Setup(self, airsimSettings):
        print("Setup")
        vehicles = airsimSettings.get("Vehicles", {})
        for drone_name, settings in vehicles.items():
            # Extracting the X, Y, Z offsets from the settings
            x_offset = settings.get("X", 0)
            y_offset = settings.get("Y", 0)
            z_offset = settings.get("Z", 0)

            # Creating a DroneData instance for each drone and adding it to the drones dictionary
            self.drones[drone_name] = DroneData(drone_name, x_offset, y_offset, z_offset)

        print("Drone setup completed. Registered # of drones:", len(self.drones))



    async def monitor_drones(self):

        while True:
            for drone in self.drones.values():
                kinematicState = self.client.simGetGroundTruthKinematics(vehicle_name=drone.drone_name)
                multiRotorState = self.client.getMultirotorState(vehicle_name=drone.drone_name)

                latitude = kinematicState.position.x_val
                longitude = kinematicState.position.y_val
                altitude = kinematicState.position.z_val
                velocity = kinematicState.linear_velocity.get_length()
                drone.x = kinematicState.position.x_val + drone.x_offset + landscape_offset_x
                drone.y = kinematicState.position.y_val + drone.y_offset + landscape_offset_y
                drone.z = kinematicState.position.z_val + drone.z_offset + landscape_offset_z
                yaw = 0#multiRotorState.kinematics_estimated.orientation.to_eularian_angles()[ 2]  # Assuming yaw needs calculation

                drone_update = UpdateDrone(drone.drone_name, latitude, longitude, altitude, velocity, drone.x, drone.y, drone.z, yaw)
                await self.broadcast(json.dumps(drone_update.__dict__))

            await asyncio.sleep(1 / 20)

    async def send_drone_creation_messages(self, websocket):
        for drone_name, drone_data in self.drones.items():
            create_drone_message = Messages.CreateDrone(
                drone_name=drone_data.drone_name,
                x=drone_data.x_offset,
                y=drone_data.y_offset,
                z=drone_data.z_offset
            )
            # Serialize the message to JSON and send it
            message_json = json.dumps(create_drone_message.__dict__)
            await websocket.send(message_json)
            print(f"Sent create drone message for {drone_name} to new client")

    async def send_current_bayes(self, websocket):
        for drone_name, drone_data in self.drones.items():
            create_drone_message = Messages.CreateDrone(
                drone_name=drone_data.drone_name,
                x=drone_data.x_offset,
                y=drone_data.y_offset,
                z=drone_data.z_offset
            )
            # Serialize the message to JSON and send it
            message_json = json.dumps(create_drone_message.__dict__)
            await websocket.send(message_json)
            print(f"Sent create drone message for {drone_name} to new client")
