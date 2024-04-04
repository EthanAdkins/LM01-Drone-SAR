import airsim
import Constants.configDrones as configDrones
import ImageProcessing.getInfo as getInfo
import time
LOCAL_IP = configDrones.LOCAL_IP
# Connect to the AirSim simulator
droneName = "0"
client = airsim.MultirotorClient(LOCAL_IP)
client.confirmConnection()
client.enableApiControl(True, droneName)
client.armDisarm(True, droneName)
client.takeoffAsync(vehicle_name=droneName).join()
client.moveToZAsync(z=-100, velocity=8, vehicle_name = droneName).join()

# Ok so the offset is 11. You have to get the absolute location of an object during runtime and divide that Unreal Coord by 100 (cm to m) then subtract offset.
positions = [
    (915.330 - 10, -517.030 - 4, -600),
    #(37.220 - 10, -644.570 - 4, -600)

    # (380.830 - 10, 223.810 - 4, -100),
    # (1.930 - 10, -155.290 - 4, -100),

    # (502.79527344 - 10, 548.28460938 - 4, -10),
    # (17.44185791 - 10, 548.29382812 - 4, -10),
    # (17.30290283 - 10, -463.49074219 - 4, -10),
    # (502.69332031 - 10, -463.41339844 - 4, -10)

    # (0.004646968378418462,  0.004564568027176327, 10),
    # ( 0.00024996666641051104, 0.004574718382269764, 10),
    # (0.0002027471053771984, -0.004475452495970286, 10),
    # (0.004555081358354454, -0.004507095470938446, 10)
]


for position in positions:
    client.moveToPositionAsync(*position, velocity = 30, vehicle_name=droneName).join()
    # Get GPS data
    time.sleep(13)
    _, lat, lon = getInfo.getDroneGPS("0", client)
    
    # Print GPS coordinates
    print("Unreal Coords: ", position)
    print("Latitude: {}".format(lat))
    print("Longitude: {}\n".format(lon))

# client.moveToGPSAsync(latitude=0.0026853068927250876, longitude=-0.00018847169112613604, altitude=100, velocity=10).join()
print("AT LOCATION")


