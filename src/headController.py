"""
followMeHeadExecution
---------------------

Vision-based object detection and robot control system for the humanoid robot TEO

Author: Alvaro Santos Garcia
Copyright: Universidad Carlos III de Madrid (C) 2025;
CopyPolicy: Released under the terms of the GNU LGPL v2.1
"""

import yarp
import math
from time import sleep
from threading import Thread
import roboticslab_kinematics_dynamics as kd
import PyKDL as kdl

DEFAULT_ROBOT = "/teoSim"
DEFAULT_PREFIX = "/followMeHeadExecution"
DEFAULT_REF_SPEED = 30.0
DEFAULT_REF_ACCELERATION = 30.0
DETECTION_DEADBAND = 0.03  # [m]
RELATIVE_INCREMENT = 2.0  # [deg]
TRANS_INCREMENT = 0.001 # [m]
ROT_INCREMENT = 0.1 # [deg]
APPROXIMATION_DEADBAND = 0.05 # [m]
ROTATION_DEADBAND = 5 # [deg]

class FollowMeHeadExecution(yarp.RFModule):
    def __init__(self, cameraDetection, aruco=None):
        super().__init__()

        # ------------- Attributes -------------
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
        self.connection_attempts = 5  # Added
        self.connection_delay = 0.5  # Added

        # -------------- Classes ------------------------
        self.cameraDetection = cameraDetection
        self.aruco = aruco

        # ------------- Thread --------------------
        self.should_stop = False
        self.thread = Thread(target=self._thread_function, daemon=True)

        self.rf = yarp.ResourceFinder()  # Added for configuration

    def stop(self):
        self.should_stop = True
        self.thread.join()
        self.cameraDetection.stop()
        self.aruco.stop()

    def configure(self, rf):
        self.rf = rf

        # ------------------- Get values -------------------
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

        # ------------------ Interface check -------------------------
        for name, interface in required_interfaces.items():
            if not interface:
                print(f"Error: Missing interface {name}")
                return False
            setattr(self, name, interface)

        self.axes = self.iPositionControl.getAxes()
        print("Configure", self.axes)
        modes = yarp.IVector(self.axes, yarp.VOCAB_CM_POSITION)
        if not self.iControlMode.setControlModes(modes):
            print("Error setting control modes")
            return False

        ref_speed = self.rf.check("ref_speed", yarp.Value(30.0)).asFloat64()
        ref_accel = self.rf.check("ref_acceleration", yarp.Value(30.0)).asFloat64()

        speeds = yarp.DVector(self.axes, ref_speed)
        accels = yarp.DVector(self.axes, ref_accel)

        self.iPositionControl.setRefSpeeds(speeds)
        self.iPositionControl.setRefAccelerations(accels)

        if not self.iCartesianControlArm.setParameter(kd.VOCAB_CC_CONFIG_FRAME, yarp.encode('cpfb')):
            print('unable to set TCP frame')
            return False

        if not self.iCartesianControlArm.setParameter(kd.VOCAB_CC_CONFIG_STREAMING_CMD, kd.VOCAB_CC_POSE):
            print('unable to preset POSE')
            return False

        if not self._verify_connection():
            return

        self.thread.start()
        return True

    def _thread_function(self):
        while not self.should_stop:
            self.moveHead(self.cameraDetection.x, self.cameraDetection.y)

            if self.aruco.move_arm:
                self.moveArm(self.cameraDetection.x, self.cameraDetection.y, self.cameraDetection.z)
            sleep(0.05)

    def _verify_connection(self):
        # ====== Often fails the first time ======
        encs = yarp.DVector(self.axes)
        for _ in range(3):
            if self.iEncoders.getEncoders(encs):
                self.currentPos = [encs[0], encs[1]]
                print(f"Connection verified. Initial position: {self.currentPos}")
                return True
            sleep(0.3)
        return False

    def moveHead(self, x, y):
        targets = yarp.DVector(self.axes)
        targets[0] = math.copysign(RELATIVE_INCREMENT, -x) if abs(x) > DETECTION_DEADBAND else 0.0
        targets[1] = math.copysign(RELATIVE_INCREMENT, y) if abs(y) > DETECTION_DEADBAND else 0.0

        return self.iPositionControl.relativeMove(targets)

    def moveArm(self, x, y, z):
        v = yarp.DVector()
        ret, state, ts = self.iCartesianControlArm.stat(v)

        H_current = kdl.Frame.Identity()
        H_current.p = kdl.Vector(v[0], v[1], v[2])
        axis_current = kdl.Vector(v[3], v[4], v[5])
        angle_current = axis_current.Normalize()
        H_current.M = kdl.Rotation.Rot(axis_current, angle_current)

        v_head = yarp.DVector()
        _, _, _ = self.iCartesianControlHead.stat(v_head)
        H_0_rgb = kdl.Frame.Identity()
        H_0_rgb.p = kdl.Vector(v_head[0], v_head[1], v_head[2])
        axis_0_rgb = kdl.Vector(v_head[3], v_head[4], v_head[5])
        angle_0_rgb = axis_0_rgb.Normalize()
        H_0_rgb.M = kdl.Rotation.Rot(axis_0_rgb, angle_0_rgb) * kdl.Rotation.RotZ(math.pi * 0.5)

        H_rgb_obj = kdl.Frame(kdl.Vector(x, y, z))

        H_0_obj = H_0_rgb * H_rgb_obj

        twist = kdl.diff(H_current, H_0_obj)
        linear = twist.vel
        angular = twist.rot

        linear_distance = linear.Normalize()
        angular_distance = angular.Normalize()

        if linear_distance < APPROXIMATION_DEADBAND and angular_distance < (ROTATION_DEADBAND * math.pi / 180):
            return False

        # Proportional scaling
        linear_step = min(linear_distance, TRANS_INCREMENT)
        angular_step = min(angular_distance, ROT_INCREMENT * math.pi / 180)

        H_next = kdl.Frame(H_current.M, H_current.p)
        H_next.p += linear * linear_step
        if angular_distance > 1e-3:
            H_next.M = kdl.Rotation.Rot(angular, angular_step) * H_current.M

        axis = H_next.M.GetRot()
        angle = axis.Normalize()
        axis *= angle

        def clip(val, limit=0.544):
            return max(min(val, limit), -limit)

        clipped_pos = kdl.Vector(
            clip(H_next.p.x()),
            clip(H_next.p.y()),
            clip(H_next.p.z())
        )

        H_next.p = clipped_pos

        xd = yarp.DVector([
            H_next.p.x(),
            H_next.p.y(),
            H_next.p.z(),
            axis.x(),
            axis.y(),
            axis.z()
        ])

        self.iCartesianControlArm.pose(xd)
        return True
