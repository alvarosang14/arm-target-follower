import math
from array import array
import yarp

DEFAULT_ROBOT = "/teoSim"
DEFAULT_PREFIX = "/followMeHeadExecution"
DEFAULT_REF_SPEED = 30.0
DEFAULT_REF_ACCELERATION = 30.0
DETECTION_DEADBAND = 0.03  # [m]
MAX_PAN = 60.0  # degrees
MAX_TILT = 30.0  # degrees


class FollowMeHeadExecution(yarp.RFModule):
    def __init__(self):
        super().__init__()
        self.headDevice = yarp.PolyDriver()
        self.iControlMode = None
        self.iEncoders = None
        self.iPositionControl = None
        self.currentPos = [0.0, 0.0]
        self.axes = 0

    def configure(self, rf):
        robot = rf.check("robot", yarp.Value(DEFAULT_ROBOT)).asString()
        local_prefix = rf.check("local", yarp.Value(DEFAULT_PREFIX)).asString()

        headOptions = yarp.Property()
        headOptions.put("device", "remote_controlboard")
        headOptions.put("remote", f"{robot}/head")
        headOptions.put("local", f"{local_prefix}/head")

        if not self.headDevice.open(headOptions):
            print("Failed to open head device")
            return False

        self.iPositionControl = self.headDevice.viewIPositionControl()
        self.iControlMode = self.headDevice.viewIControlMode()
        self.iEncoders = self.headDevice.viewIEncoders()

        if not all([self.iPositionControl, self.iControlMode, self.iEncoders]):
            print("Failed to view one or more interfaces")
            return False

        self.axes = self.iPositionControl.getAxes()
        if self.axes != 2:
            print(f"Expected 2 axes, got {self.axes}")
            return False

        # Configurar modos de control (versión compatible)
        modes = yarp.IVector(self.axes)
        for i in range(self.axes):
            modes[i] = yarp.VOCAB_CM_POSITION
        if not self.iControlMode.setControlModes(modes):
            print("Failed to set position control mode")
            return False

        # Configurar velocidades y aceleraciones
        speeds = yarp.DVector(self.axes, DEFAULT_REF_SPEED)
        accels = yarp.DVector(self.axes, DEFAULT_REF_ACCELERATION)
        self.iPositionControl.setRefSpeeds(speeds)
        self.iPositionControl.setRefAccelerations(accels)

        return self._updateCurrentPosition()

    def _updateCurrentPosition(self):
        encs = yarp.DVector(self.axes)
        if not self.iEncoders:
            print("Error: Interfaz IEncoders no disponible")
            return False

        if self.iEncoders.getEncoders(encs):
            self.currentPos = [encs[0], encs[1]]
            print(f"Posición actual: {self.currentPos}")
            return True
        else:
            print("Error leyendo encoders. Verifica:")
            print("- Si el dispositivo está realmente conectado")
            print("- Si los encoders están habilitados en el hardware/simulador")
            return False

    def updateTarget(self, x, y, z):
        if not self._updateCurrentPosition():
            return False

        pan_error = -x * 0.5
        tilt_error = y * 0.5

        if abs(pan_error) < DETECTION_DEADBAND:
            pan_error = 0
        if abs(tilt_error) < DETECTION_DEADBAND:
            tilt_error = 0

        target_pan = self.currentPos[0] + math.degrees(pan_error)
        target_tilt = self.currentPos[1] + math.degrees(tilt_error)

        target_pan = max(min(target_pan, MAX_PAN), -MAX_PAN)
        target_tilt = max(min(target_tilt, MAX_TILT), -MAX_TILT)

        print(f"Moving head to pan={target_pan:.1f}°, tilt={target_tilt:.1f}°")

        targets = yarp.DVector(self.axes)
        targets[0] = target_pan
        targets[1] = target_tilt
        return self.iPositionControl.positionMove(targets)  # Eliminar .data()

    def getPeriod(self):
        return 0.1

    def updateModule(self):
        return True

    def interruptModule(self):
        return self.close()

    def close(self):
        if self.headDevice.isValid():
            self.headDevice.close()
        return True