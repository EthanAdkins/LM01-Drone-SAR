import asyncio
import json
import websockets

# Message classes remain the same
class BaseMessage:
    def __init__(self, message_type):
        self.message_type = message_type

    def to_json(self):
        return json.dumps(self.__dict__)

class DroneUpdate(BaseMessage):
    def __init__(self, drone_name, latitude, longitude, altitude, velocity, x, y, z, yaw):
        super().__init__(message_type="DroneUpdate")
        self.drone_name = drone_name
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.velocity = velocity
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

# Store clients and drone data
clients = {"drone_swarms": set(), "minimaps": set()}
drones = {}  # Stores the latest update of each drone

async def broadcast_to_minimaps(message):
    if clients["minimaps"]:
        await asyncio.wait([client.send(message) for client in clients["minimaps"]])

async def send_current_drones_state(websocket):
    """Send the current state of all drones to a newly connected minimap."""
    for drone_update in drones.values():
        await websocket.send(drone_update.to_json())

async def handle_client_type(websocket, message):
    client_type = message.get('client_type')
    if client_type == 'drone_swarm':
        clients['drone_swarms'].add(websocket)
        print(f"Drone swarm client connected: {websocket.remote_address}")
    elif client_type == 'minimap':
        clients['minimaps'].add(websocket)
        await send_current_drones_state(websocket)
        print(f"Minimap client connected: {websocket.remote_address}")

async def update_drone_data(message_data):
    """Update the stored data for a drone."""
    drone_update = DroneUpdate(**message_data)
    drones[drone_update.drone_name] = drone_update

async def unregister(websocket):
    if websocket in clients['drone_swarms']:
        clients['drone_swarms'].discard(websocket)
        print(f"Drone swarm client disconnected: {websocket.remote_address}")
    if websocket in clients['minimaps']:
        clients['minimaps'].discard(websocket)
        print(f"Minimap client disconnected: {websocket.remote_address}")

async def websocket_server(websocket, path):
    try:
        # Process drone updates or registrations
        async for message in websocket:
            message_data = json.loads(message)
            if 'message_type' == "authentication":
                await handle_client_type(websocket, message_data)
                break
            if websocket in clients['drone_swarms']:
                if message_data['message_type'] == 'DroneUpdate':
                    await update_drone_data(message_data)
                    await broadcast_to_minimaps(message)  # Forward updated data to all minimaps
                if message_data['message_type'] == 'CreateDrone':
                    await broadcast_to_minimaps(message)  # Forward updated data to all minimaps
    finally:
        await unregister(websocket)

async def main():
    async with websockets.serve(websocket_server, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
