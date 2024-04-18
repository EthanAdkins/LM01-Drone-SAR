
class DroneData():
    def __init__(self, drone_name, x_offset, y_offset, z_offset):

        #Setup initial
        self.drone_name = drone_name
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset

        #Setup active data
        self.x = x_offset
        self.y = y_offset
        self.z = z_offset
        self.yaw = 0