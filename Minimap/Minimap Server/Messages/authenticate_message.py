from Messages import BaseMessage

class AuthenticateMessage(BaseMessage):
    def __init__(self, client_type):
        super().__init__(message_type="authentication")
        self.client_type = client_type
