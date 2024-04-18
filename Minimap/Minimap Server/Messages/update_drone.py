from Messages import BaseMessage

class UpdateDrone(BaseMessage):
    def __init__(self, drone_name, latitude, longitude, altitude, velocity, x, y, z, yaw):
        super().__init__(message_type="UpdateDrone")
        self.drone_name = drone_name
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.velocity = velocity
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw