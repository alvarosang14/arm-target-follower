import yarp

class RGBDetection:
    def __init__(self, remote_port, head_module):
        self.remote_port = remote_port
        self.local_port = yarp.Port()
        self.head_module = head_module

        # Inicializa YARP
        yarp.Network.init()
        if not yarp.Network.checkNetwork():
            raise RuntimeError("YARP server no encontrado. Lanza 'yarpserver' primero.")

        # Configurar puertos
        self.setup_camera_port()
        self.update_camera_view()

    def setup_camera_port(self):
        """Configura el puerto YARP para recibir coordenadas"""
        self.local_port.open("/client/rgbd/state:i")

        if not yarp.Network.connect(self.remote_port, "/client/rgbd/state:i"):
            print(f"No se pudo conectar a {self.remote_port}")

    def update_camera_view(self):
        """Lee coordenadas X, Y, Z en bucle"""
        try:
            while True:
                bottle = yarp.Bottle()
                self.local_port.read(bottle)

                if bottle.size() == 3:
                    x = bottle.get(0).asFloat64()
                    y = bottle.get(1).asFloat64()
                    z = bottle.get(2).asFloat64()
                    print(f"Detectado centro a coordenadas: x={x:.2f}, y={y:.2f}, z={z:.2f}")

                    self.head_module.updateTarget(x, y, z)

                else:
                    print("Mensaje inesperado:", bottle.toString())

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Detiene el sistema y cierra puertos"""
        if self.local_port.isOpen():
            self.local_port.close()
        yarp.Network.fini()
