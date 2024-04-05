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
    # High altitude Locations
    (111.70778306 - 10, -496.9737205 - 4, -600),
    (174.160 - 10, -499.860 - 4, -600),
    (1156.810 - 10, -551.860 - 4, -600),
    (953.190 - 10, 529.640  - 4, -600),
    (1013.340 - 10,  411.500 - 4, -600),
    (1055.490 - 10, 308.750  - 4, -600),
    (1045.910 - 10, 127.280 - 4, -600),
    ( 983.750- 10,  -68.170 - 4, -600)

    # Gazebos
    # (807.270 - 10, 171.480 - 4, -600),
    # (190.480 - 10, -651.720 - 4, -600),
    # (-264.190 - 10, -118.170 - 4, -600),
    # (-169.890 - 10, 671.360 - 4, -600),
    # (1230.970 - 10, -681.620 - 4, -600),
    # (244.350 - 10, -15.800 - 4, -600),
    # (866.070 - 10, 203.940 - 4, -600),
    # (1608.180 - 10, 221.450 - 4, -600),
    # (931.070 - 10, 755.010 - 4, -600),
    # (811.890 - 10, -370.910 - 4, -600),
    # (1127.440 - 10, -66.670 - 4, -600),
    # (1400.450 - 10, -120.890 - 4, -600),
    # (1162.430 - 10, -777.630 - 4, -600),
    # (1302.470 - 10, -464.440 - 4, -600),
    # (548.720 - 10, -826.300 - 4, -600)




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


