import yarp
import math
from time import sleep
import roboticslab_kinematics_dynamics as kd
import PyKDL as kdl

DEFAULT_ROBOT = "/teoSim"
DEFAULT_PREFIX = "/followMeHeadExecution"
DEFAULT_REF_SPEED = 30.0
DEFAULT_REF_ACCELERATION = 30.0
DETECTION_DEADBAND = 0.03  # [m]
RELATIVE_INCREMENT = 2.0  # [deg]
TRANS_INCREMENT = 0.001 # [m]
ROT_INCREMENT = 0.1 # [deg
APPROXIMATION_DEADBAND = 0.05 # [m]
ROTATION_DEADBAND = 5 # [deg]

class FollowMeHeadExecution(yarp.RFModule):
    def __init__(self):
        super().__init__()

        # ------------- Atributos -------------
        self.headDevice = yarp.PolyDriver()
        self.cartesianHeadDevice = yarp.PolyDriver()
        self.cartesianArmDevice = yarp.PolyDriver()
        self.iControlMode = None
        self.iEncoders = None
        self.iPositionControl = None
        self.iCartesianControlHead = None
        self.iCartesianControlArm = None
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

        headOptionsCartesian = yarp.Property()
        headOptionsCartesian.put("device", "CartesianControlClient")
        headOptionsCartesian.put("cartesianRemote", f"{robot}/head/CartesianControl")
        headOptionsCartesian.put("cartesianLocal", f"{local_prefix}/head/CartesianControl")

        armOptionsCartesian = yarp.Property()
        armOptionsCartesian.put("device", "CartesianControlClient")
        armOptionsCartesian.put("cartesianRemote", f"{robot}/rightArm/CartesianControl")
        armOptionsCartesian.put("cartesianLocal", f"{local_prefix}/rightArm/CartesianControl")

        if not self.headDevice.open(headOptions):
            print('head not connected')
            return False

        if not self.cartesianHeadDevice.open(headOptionsCartesian):
            print('cartesian head not connected')
            return False

        if not self.cartesianArmDevice.open(armOptionsCartesian):
            print('cartesian arm not connected')
            return False

        # ------------------ Interfaces ------------------
        required_interfaces = {
            'iPositionControl': self.headDevice.viewIPositionControl(),
            'iControlMode': self.headDevice.viewIControlMode(),
            'iEncoders': self.headDevice.viewIEncoders(),
            'iCartesianControlHead': kd.viewICartesianControl(self.cartesianHeadDevice),
            'iCartesianControlArm': kd.viewICartesianControl(self.cartesianArmDevice)
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

        v = yarp.DVector()
        ret, state, ts = self.iCartesianControlArm.stat(v)

        H_0_tcp = kdl.Frame.Identity()
        H_0_tcp.p = kdl.Vector(v[0], v[1], v[2])
        axis_0_tcp = kdl.Vector(v[3], v[4], v[5])
        angle_0_tcp = axis_0_tcp.Normalize()
        H_0_tcp.M = kdl.Rotation.Rot2(axis_0_tcp, angle_0_tcp)
        self.H_0_tcp = H_0_tcp

        if not self.iCartesianControlArm.setParameter(kd.VOCAB_CC_CONFIG_FRAME, yarp.encode('cpfb')):
            print('unable to set TCP frame')
            return False

        if not self.iCartesianControlArm.setParameter(kd.VOCAB_CC_CONFIG_STREAMING_CMD, kd.VOCAB_CC_POSE):
            print('unable to preset POSE')
            return False

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

    def moveHead(self, x, y):
        if not self.isFollowing:
            print("Modo following desactivado")
            return False

        targets = yarp.DVector(self.axes)
        targets[0] = math.copysign(RELATIVE_INCREMENT, -x) if abs(x) > DETECTION_DEADBAND else 0.0
        targets[1] = math.copysign(RELATIVE_INCREMENT, y) if abs(y) > DETECTION_DEADBAND else 0.0

        return self.iPositionControl.relativeMove(targets)

    def moveArm(self, x, y, z):
        if not self.isFollowing:
            print("Modo following desactivado")
            return False

        v = yarp.DVector()
        ret, state, ts = self.iCartesianControlHead.stat(v)

        H_0_rgb = kdl.Frame.Identity()
        H_0_rgb.p = kdl.Vector(v[0], v[1], v[2])
        axis_0_rgb = kdl.Vector(v[3], v[4], v[5])
        angle_0_rgb = axis_0_rgb.Normalize()
        H_0_rgb.M = kdl.Rotation.Rot2(axis_0_rgb, angle_0_rgb) * kdl.Rotation.RotZ(math.pi * 0.5)

        H_rgb_obj = kdl.Frame(kdl.Vector(x, y, z))

        H_0_obj = H_0_rgb * H_rgb_obj

        # modo lineal
        #H_tcp_obj = self.H_tcp_0 * H_0_obj
        #p = H_tcp_obj.p
        #distance = p.Normalize()

        twist = kdl.diff(self.H_0_tcp, H_0_obj)
        p = twist.vel
        distance = p.Normalize()
        rot = twist.rot
        angle = rot.Normalize()

        #H_tcp_step = kdl.Frame.Identity()

        if distance >= APPROXIMATION_DEADBAND or angle >= (ROTATION_DEADBAND * math.pi / 180):
            p *= TRANS_INCREMENT
            rot *= ROT_INCREMENT * math.pi / 180

            #H_tcp_step.p = p
            #H_tcp_step.M = kdl.Rotation.Rot(rot, rot.Norm())

        self.H_0_tcp.p += p
        axis = self.H_0_tcp.M.GetRot()
        xd = yarp.DVector([self.H_0_tcp.p.x(), self.H_0_tcp.p.y(), self.H_0_tcp.p.z(), axis.x(), axis.y(), axis.z()])

        #self.H_0_tcp *= H_tcp_step
        #axis = self.H_0_tcp.M.GetRot()
        #xd = yarp.DVector([self.H_0_tcp.p.x(), self.H_0_tcp.p.y(), self.H_0_tcp.p.z(), axis.x(), axis.y(), axis.z()])

        self.iCartesianControlArm.pose(xd)

        return True

    def _CurrentPosition(self):
        encs = yarp.DVector(self.axes)
        if not self.iEncoders.getEncoders(encs):
            print("Error al leer encoders")
            return False

        self.currentPos = [encs[0], encs[1]]
        print(f"Posición actualizada: pan={self.currentPos[0]:.2f}°, tilt={self.currentPos[1]:.2f}°")
        return True