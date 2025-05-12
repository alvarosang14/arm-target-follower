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
from src.main.cameraDetection import RGBDetection
from headController import FollowMeHeadExecution


@begin.start(auto_convert=True)
@begin.logging
def main(remote_port: 'Remote port running the AravisGigE grabber' = '/rgbdDetection/state:o'):

    yarp.Network.init()

    if not yarp.Network.checkNetwork():
        yarp.yError("found no yarp network (try running \"yarpserver &\"")
        return

    rf = yarp.ResourceFinder()
    rf.setDefaultContext("followMeHeadExecution")
    rf.setDefaultConfigFile("followMeHeadExecution.ini")
    rf.configure(sys.argv)

    mod = FollowMeHeadExecution()
    mod.configure(rf)

    RGBDetection(remote_port, mod)

    if rf.check("help"):
        return mod.runModule(rf)

    yarp.yInfo(f'Run "{sys.argv[0]} --help" for options')
    yarp.yInfo(f"{sys.argv[0]} checking for yarp network...")

