from Messages import BaseMessage

class UpdateDroneCluster(BaseMessage):
    def __init__(self, cluster_name, drone_names):
        super().__init__(message_type="UpdateDroneCluster")
        self.cluster_name = cluster_name
        self.drone_names = drone_names