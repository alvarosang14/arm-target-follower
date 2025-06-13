import yarp

class ArucoDetection(yarp.BottleCallback):
    def __init__(self, remote_port):
        super().__init__()

        self.remote_port = remote_port
        self.local_port = yarp.BufferedPortBottle()

        # Set up ports
        self.setup_camera_port()

        self.move_arm = False

    def setup_camera_port(self):
        """Sets up the YARP port to receive coordinates"""
        self.local_port.open("/client/rgb/state:i")

        if not yarp.Network.connect(self.remote_port, "/client/rgb/state:i"):
            print(f"Could not connect to {self.remote_port}")

        self.local_port.useCallback(self)

    def onRead(self, bottle, reader):
        for i in range(bottle.size()):
            text = bottle.get(i).asDict().find("text").asInt32()
            print("text", text)

            if text == 20:
                print("20 detected")
                self.move_arm = True
        print("Aruco detected")

    def stop(self):
        """Stops the system and closes ports"""
        if self.local_port.isOpen():
            self.local_port.interrupt()
            self.local_port.close()
