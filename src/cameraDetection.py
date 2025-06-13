import yarp

class RGBDetection(yarp.BottleCallback):
    def __init__(self, remote_port):
        super().__init__()

        self.remote_port = remote_port
        self.local_port = yarp.BufferedPortBottle()

        self.x = 0
        self.y = 0
        self.z = 0

        # Set up ports
        self.setup_camera_port()

    def setup_camera_port(self):
        """Sets up the YARP port to receive coordinates"""
        self.local_port.open("/client/rgbd/state:i")

        if not yarp.Network.connect(self.remote_port, "/client/rgbd/state:i"):
            print(f"Could not connect to {self.remote_port}")

        self.local_port.useCallback(self)

    def onRead(self, bottle, reader):
        """Continuously reads X, Y, Z coordinates"""
        if bottle.size() == 3:
            self.x = bottle.get(0).asFloat64()
            self.y = bottle.get(1).asFloat64()
            self.z = bottle.get(2).asFloat64()

    def stop(self):
        """Stops the system and closes ports"""
        if self.local_port.isOpen():
            self.local_port.interrupt()
            self.local_port.close()
