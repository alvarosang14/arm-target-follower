#!/usr/bin/env python3

"""
GrabberControls2Gui
---------------------

Simple GUI for controlling GigE cameras using Aravis and YARP

Author: Alvaro Santos Garcia
Copyright: Universidad Carlos III de Madrid (C) 2025;
CopyPolicy: Released under the terms of the GNU GPL v2.0.
"""

import sys
import begin
import yarp
import signal
import time
import logging
from cameraDetection import RGBDetection
from headController import FollowMeHeadExecution


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

    mod = FollowMeHeadExecution()

    if not mod.configure(rf):
        return

    detection = RGBDetection(remote_port, mod)

    should_stop = False

    def signal_handler(signum, frame):
        global should_stop
        should_stop = True

    signal.signal(signal.SIGINT, signal_handler)

    while not should_stop:
        time.sleep(0.1)

    yarp.Network.fini()
