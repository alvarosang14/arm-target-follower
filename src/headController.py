import yarp
import math
from time import sleep
from array import array

DEFAULT_ROBOT = "/teoSim"
DEFAULT_PREFIX = "/followMeHeadExecution"
DEFAULT_REF_SPEED = 30.0
DEFAULT_REF_ACCELERATION = 30.0
DETECTION_DEADBAND = 0.03  # [m]
RELATIVE_INCREMENT = 2.0  # [DEG]



class FollowMeHeadExecution(yarp.RFModule):
    def __init__(self):
        super().__init__()

        # ------------- Atributos -------------
        self.headDevice = yarp.PolyDriver()
        self.iControlMode = None
        self.iEncoders = None
        self.iPositionControl = None
        self.currentPos = [0.0, 0.0]
        self.axes = 0
        self.isFollowing = False
        self.connection_attempts = 5  # Añadido
        self.connection_delay = 0.5  # Añadido

        self.rf = yarp.ResourceFinder()  # Añadido para configuración

    def configure(self, rf):
        self.rf = rf

        # ------------------- Sacamo valores -------------------
        robot = self.rf.check("robot", yarp.Value("/teoSim")).asString()
        local_prefix = self.rf.check("local", yarp.Value("/followMeHeadExecution")).asString()

        headOptions = yarp.Property()
        headOptions.put("device", "remote_controlboard")
        headOptions.put("remote", f"{robot}/head")
        headOptions.put("local", f"{local_prefix}/head")
        headOptions.put("writeStrict", "on")

        # ============= ¿ Sigue siendo necesario ? =============
        for attempt in range(self.connection_attempts):
            if self.headDevice.open(headOptions):
                break
            print(f"Intento {attempt + 1} fallido, reintentando...")
            sleep(self.connection_delay)
        else:
            print("Error: No se pudo conectar al dispositivo de cabeza")
            return False

        # ------------------ Interfaces ------------------
        required_interfaces = {
            'iPositionControl': self.headDevice.viewIPositionControl(),
            'iControlMode': self.headDevice.viewIControlMode(),
            'iEncoders': self.headDevice.viewIEncoders()
        }

        # ------------------ Comprobacion de interfaces -------------------------
        for name, interface in required_interfaces.items():
            if not interface:
                print(f"Error: Falta interfaz {name}")
                return False
            setattr(self, name, interface)

        self.axes = self.iPositionControl.getAxes()
        modes = yarp.IVector(self.axes, yarp.VOCAB_CM_POSITION)
        if not self.iControlMode.setControlModes(modes):
            print("Error al configurar modos de control")
            return False

        ref_speed = self.rf.check("ref_speed", yarp.Value(30.0)).asFloat64()
        ref_accel = self.rf.check("ref_acceleration", yarp.Value(30.0)).asFloat64()

        speeds = yarp.DVector(self.axes, ref_speed)
        accels = yarp.DVector(self.axes, ref_accel)

        self.iPositionControl.setRefSpeeds(speeds)
        self.iPositionControl.setRefAccelerations(accels)

        return self._verify_connection()

    def _verify_connection(self):
        # ====== Suele fallar a la priema ======
        encs = yarp.DVector(self.axes)
        for _ in range(3):
            if self.iEncoders.getEncoders(encs):
                self.currentPos = [encs[0], encs[1]]
                print(f"Conexión verificada. Posición inicial: {self.currentPos}")
                return True
            sleep(0.3)
        return False

    def updateTarget(self, x, y, z):
        if not self.isFollowing:
            print("Modo following desactivado")
            return False


        targets = yarp.DVector(self.axes)
        targets[0] = math.copysign(RELATIVE_INCREMENT, -x) if abs(x) > DETECTION_DEADBAND else 0.0
        targets[1] = math.copysign(RELATIVE_INCREMENT, y) if abs(y) > DETECTION_DEADBAND else 0.0

        return self.iPositionControl.relativeMove(targets)

    def _CurrentPosition(self):
        encs = yarp.DVector(self.axes)
        if not self.iEncoders.getEncoders(encs):
            print("Error al leer encoders")
            return False

        self.currentPos = [encs[0], encs[1]]
        print(f"Posición actualizada: pan={self.currentPos[0]:.2f}°, tilt={self.currentPos[1]:.2f}°")
        return True