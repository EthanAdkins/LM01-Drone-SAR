import json
import asyncio
import websocket_server

def get_vehicles_from_settings(settings_path):
    """Load vehicle names from the AirSim settings.json file."""
    try:
        with open(settings_path) as f:
            settings = json.load(f)
            return list(settings.get("Vehicles", {}).keys())
    except FileNotFoundError:
        print(f"Settings file not found: {settings_path}")
        return []

async def main():

    #Logging
    print("Starting server.")

    #Get settings path
    settings_path = 'C:/Users/Admin/Documents/AirSim/settings.json'  # Update this path as needed

    #Load settings to json
    with open(settings_path) as f:
        settings = json.load(f)

        # Start the WebSocket server and pass the drone monitoring the list of vehicles
        await websocket_server.start_server(settings)

if __name__ == "__main__":
    asyncio.run(main())
