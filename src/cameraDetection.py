import yarp

class RGBDetection(yarp.BottleCallback):
    def __init__(self, remote_port):
        super().__init__()

        self.remote_port = remote_port
        self.local_port = yarp.BufferedPortBottle()

        self.x = 0
        self.y = 0
        self.z = 0

        # Configurar puertos
        self.setup_camera_port()

    def setup_camera_port(self):
        """Configura el puerto YARP para recibir coordenadas"""
        self.local_port.open("/client/rgbd/state:i")

        if not yarp.Network.connect(self.remote_port, "/client/rgbd/state:i"):
            print(f"No se pudo conectar a {self.remote_port}")

        self.local_port.useCallback(self)

    def onRead(self, bottle, reader):
        """Lee coordenadas X, Y, Z en bucle"""
        if bottle.size() == 3:
            self.x = bottle.get(0).asFloat64()
            self.y = bottle.get(1).asFloat64()
            self.z = bottle.get(2).asFloat64()
            #print(f"Detectado centro a coordenadas: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}")

            #self.head_module.moveHead(x, y)
            #self.head_module.moveArm(x, y, z)

    def stop(self):
        """Detiene el sistema y cierra puertos"""
        if self.local_port.isOpen():
            self.local_port.interrupt()
            self.local_port.close()
