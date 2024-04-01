from Messages import BaseMessage

class UpdateDroneCluster(BaseMessage):
    def to_dict(self):
        """Converts the instance into a dictionary."""
        return {
            "message_type": self.message_type,  # Assuming BaseMessage sets this attribute
            "cluster_name": self.cluster_name,
            "drone_names": self.drone_names,
        }

    def __init__(self, cluster_name, drone_names):
        super().__init__(message_type="UpdateDroneCluster")
        self.cluster_name = cluster_name
        self.drone_names = drone_names