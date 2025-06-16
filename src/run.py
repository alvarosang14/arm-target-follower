#!/usr/bin/env python3

"""
followMeHeadExecution
---------------------

Vision-based object detection and robot control system for the humanoid robot TEO

Author: Alvaro Santos Garcia
Copyright: Universidad Carlos III de Madrid (C) 2025;
CopyPolicy: Released under the terms of the GNU LGPL v2.1
"""

import sys
import begin
import yarp
import signal
import time
from cameraDetection import RGBDetection
from headController import FollowMeHeadExecution
from arucoDetection import ArucoDetection


@begin.start(auto_convert=True)
@begin.logging
def main(remote_port: 'Remote port running the AravisGigE grabber' = '/rgbdDetection/state:o'):
    yarp.Network.init()

    if not yarp.Network.checkNetwork():
        yarp.yError("found no yarp network (try running \"yarpserver &\")")
        return

    rf = yarp.ResourceFinder()
    rf.setDefaultContext("followMeHeadExecution")
    rf.setDefaultConfigFile("head.ini")
    rf.configure(sys.argv)

    detection = RGBDetection(remote_port)
    aruco = ArucoDetection('/rgbDetection/state:o')

    mod = FollowMeHeadExecution(detection, aruco)

    if not mod.configure(rf):
        print("Error configuring")
        return

    should_stop = False

    def signal_handler(signum, frame):
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, signal_handler)

    while not should_stop:
        time.sleep(0.1)

    mod.stop()

    yarp.Network.fini()
