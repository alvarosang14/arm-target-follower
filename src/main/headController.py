# -*- coding: utf-8 -*-

import math
from array import array

import yarp

DEFAULT_ROBOT = "/teoSim"
DEFAULT_PREFIX = "/followMeHeadExecution"
DEFAULT_REF_SPEED = 30.0
DEFAULT_REF_ACCELERATION = 30.0

DETECTION_DEADBAND = 0.03  # [m]
RELATIVE_INCREMENT = 2.0   # [deg]

headZeros = [0.0, 0.0]ppp

class FollowMeHeadExecution(yarp.RFModule):

    def __init__(self):
        super().__init__()
        self.headDevice = yarp.PolyDriver()
        self.iControlMode = None
        self.iEncoders = None
        self.iPositionControl = None

    def configure(self, rf):
        robot = rf.check("robot", yarp.Value(DEFAULT_ROBOT)).asString()

        if rf.check("help"):
            yarp.yInfo("FollowMeHeadExecution options:")
            yarp.yInfo("\t--help (this help)\t--from [file.ini]\t--context [path]")
            yarp.yInfo(f"\t--robot: {robot} [{DEFAULT_ROBOT}]")
            return False

        headOptions = yarp.Property()
        headOptions.put("device", "remote_controlboard")
        headOptions.put("remote", robot + "/head")
        headOptions.put("local", DEFAULT_PREFIX + "/head")

        self.iPositionControl = self.headDevice.view(yarp.IPositionControl)

        if not self.headDevice.open(headOptions):
            yarp.yError("Failed to open head device")
            return False

        if self.iPositionControl is None:
            yarp.yError("Failed to view some device interfaces")
            return False

        if not self.iControlMode.setControlModes([yarp.VOCAB_CM_POSITION] * 2):
            yarp.yError("Failed to set position control mode")
            return False

        if not self.iPositionControl.setRefSpeeds([DEFAULT_REF_SPEED] * 2):
            yarp.yError("Failed to set reference speeds")
            return False

        if not self.iPositionControl.setRefAccelerations([DEFAULT_REF_ACCELERATION] * 2):
            yarp.yError("Failed to set reference accelerations")
            return False

        return True

    def getPeriod(self):
        return 0.1  # [s]

    def interruptModule(self):
        return self.stop()

    def close(self):
        self.headDevice.close()
        return True

    def updateTarget(self, x, y, z):

        if abs(x) > DETECTION_DEADBAND or abs(y) > DETECTION_DEADBAND:
            target = [
                math.copysign(RELATIVE_INCREMENT, -x) if abs(x) > DETECTION_DEADBAND else 0.0,
                math.copysign(RELATIVE_INCREMENT, y) if abs(y) > DETECTION_DEADBAND else 0.0
            ]

            yarp.yDebug(f"Received coords x={x}, y={y}, z={z} || moving head: {target}")

            if not self.iPositionControl.relativeMove(target):
                yarp.yError("Failed to move head")
        else:
            yarp.yDebug(f"Coords within deadband x={x}, y={y}, z={z}")

    def getOrientationAngle(self):
        angle = yarp.DVector(1)
        if not self.iEncoders.getEncoder(0, angle.data()):
            yarp.yError("Failed to get head orientation encoder value")
        return angle[0]

    def stop(self):
        yarp.yInfo("Received stop command")

        if not self.iPositionControl.stop():
            yarp.yError("Failed to stop head")
            return False

        return True
