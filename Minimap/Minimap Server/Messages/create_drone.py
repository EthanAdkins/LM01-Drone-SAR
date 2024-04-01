from Messages import BaseMessage

class CreateDrone(BaseMessage):
    def __init__(self, drone_name, x, y, z):
        super().__init__(message_type="CreateDrone")
        self.drone_name = drone_name
        self.x = x
        self.y = y
        self.z = z
