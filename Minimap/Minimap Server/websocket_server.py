import asyncio
import json
import websockets
from drone_monitor import DroneMonitor  # Ensure this path is correct

connected = set()


async def broadcast(message):
    if connected:
        await asyncio.wait([client.send(message) for client in connected])


async def register(websocket, drone_monitor):
    connected.add(websocket)
    # Now calling the function to send CreateDrone messages for each drone directly
    await drone_monitor.send_drone_creation_messages(websocket)


async def unregister(websocket):
    connected.remove(websocket)


async def websocket_server(websocket, path, drone_monitor):
    await register(websocket, drone_monitor)
    try:
        await websocket.wait_closed()
    finally:
        await unregister(websocket)


async def start_server(settingsFile):
    # Create and setup the drone monitor
    drone_monitor = DroneMonitor(broadcast)
    drone_monitor.Setup(settingsFile)  # Now passing the entire settings file to Setup

    # Start monitoring drones in a background task
    monitor_task = asyncio.create_task(drone_monitor.monitor_drones())

    # Serve the WebSocket, passing drone_monitor to the server function
    async with websockets.serve(lambda ws, path: websocket_server(ws, path, drone_monitor), "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    with open('C:/Users/Admin/Documents/AirSim/settings.json') as f:
        settingsFile = json.load(f)
    print("Current Vehicles:", list(settingsFile["Vehicles"].keys()))

    asyncio.run(main(settingsFile))
