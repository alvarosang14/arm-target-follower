import yarp

class ArucoDetection(yarp.BottleCallback):
    def __init__(self, remote_port):
        super().__init__()

        self.remote_port = remote_port
        self.local_port = yarp.BufferedPortBottle()

        # Configurar puertos
        self.setup_camera_port()

    def setup_camera_port(self):
        """Configura el puerto YARP para recibir coordenadas"""
        self.local_port.open("/client/rgb/state:i")

        if not yarp.Network.connect(self.remote_port, "/client/rgb/state:i"):
            print(f"No se pudo conectar a {self.remote_port}")

        self.local_port.useCallback(self)

    def onRead(self, bottle, reader):
        print(f"Detectado aruco")

    def stop(self):
        """Detiene el sistema y cierra puertos"""
        if self.local_port.isOpen():
            self.local_port.interrupt()
            self.local_port.close()
