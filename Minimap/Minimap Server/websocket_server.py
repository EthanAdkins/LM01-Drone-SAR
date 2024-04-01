import asyncio
import json
import websockets

from Messages.update_drone_cluster import UpdateDroneCluster
from drone_monitor import DroneMonitor  # Ensure this path is correct

# Define separate sets for different types of clients
minimap_clients = set()
swarm_clients = set()


async def broadcast_to_all(message):
    await asyncio.wait([client.send(message) for client in minimap_clients.union(swarm_clients)])


async def broadcast_to_minimap(message):
    if minimap_clients:
        await asyncio.wait([client.send(message) for client in minimap_clients])


async def broadcast_to_swarm(message):
    if swarm_clients:
        await asyncio.wait([client.send(message) for client in swarm_clients])


async def register(websocket, drone_monitor):
    print(f"Registering new connection from {websocket.remote_address}")
    # Initially, just add to a general connected set; categorization happens upon authentication
    # await drone_monitor.send_drone_creation_messages(websocket)


async def unregister(websocket):
    print(f"Unregistering connection from {websocket.remote_address}")
    minimap_clients.discard(websocket)
    swarm_clients.discard(websocket)  # Remove from both sets to ensure clean up


async def handle_authentication(websocket, data, drone_monitor):
    client_type = data.get("client_type")
    if client_type == "minimap":
        minimap_clients.add(websocket)
        print(f"Minimap client connected: {websocket.remote_address}")
        await drone_monitor.send_drone_creation_messages(websocket)
    elif client_type == "swarm":
        swarm_clients.add(websocket)
        print(f"Swarm client connected: {websocket.remote_address}")
    else:
        print(f"Unknown client type: {client_type}")


async def process_message(websocket, message, drone_monitor):
    try:
        data = json.loads(message)
        message_type = data.get("message_type")

        if message_type == "authentication":
            await handle_authentication(websocket, data, drone_monitor)
        elif message_type == "UpdateClusters":  # Handle the new message type
            cluster_name = data.get("cluster_name")
            drone_names = data.get("drone_names")
            # Construct the update_drone_cluster message
            print("Got cluster update")
            update_message = UpdateDroneCluster(cluster_name, drone_names)
            # Convert the update message to JSON for transmission
            update_message_json = json.dumps(update_message.to_dict())
            # Broadcast to minimap clients
            await broadcast_to_minimap(update_message_json)
        else:
            print(f"Unhandled message type: {message_type}")
    except json.JSONDecodeError as e:
        print(f"Error decoding message: {e}")



async def websocket_server(websocket, path, drone_monitor):
    await register(websocket, drone_monitor)
    try:
        async for message in websocket:
            print(f"Received message from {websocket.remote_address}: {message}")
            await process_message(websocket, message, drone_monitor)
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed with {websocket.remote_address}, reason: {e}")
    except Exception as e:
        print(f"Error processing message from {websocket.remote_address}: {e}")
    finally:
        await unregister(websocket)


async def start_server(settingsFile):
    drone_monitor = DroneMonitor(broadcast_to_minimap)
    drone_monitor.Setup(settingsFile)

    # Start monitoring drones in a background task
    monitor_task = asyncio.create_task(drone_monitor.monitor_drones())

    async with websockets.serve(
            lambda ws, path: websocket_server(ws, path, drone_monitor),
            "0.0.0.0",
            8765,
            ping_interval=10,  # Send a ping every 10 seconds
            ping_timeout=30  # Timeout if no pong is received within 30 seconds
    ):
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    with open('C:/Users/Admin/Documents/AirSim/settings.json') as f:
        settingsFile = json.load(f)
    print("Current Vehicles:", list(settingsFile["Vehicles"].keys()))

    asyncio.run(start_server(settingsFile))
