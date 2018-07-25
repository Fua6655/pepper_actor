#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import argparse
import sys
import numpy
import Image
import almath
import time
import math


def main(session):
    """
    This example alocate pepper in world coordinate
    """
    # Get the services ALNavigation and ALMotion.
    navigation_service = session.service("ALNavigation")
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")

    # Wake up robot
    motion_service.wakeUp()

	# Send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.5)
    #print "Standing"
    

    # moveAlong
    time.sleep(1)
    #result = navigation_service.moveAlong(["Composed", ["Holonomic", ["Line", [2.0, 0.0]], 0.0, 5.0], ["Holonomic", ["Line", [1, 0.0]], 0.0, 10.0]])
    #print "Navigation result", result

    # moveAlong Circle
    time.sleep(1)
    result = navigation_service.moveAlong(["Holonomic", ["Circle", [2.0, 2.0],2.0],2.0, 5.0])
    print "Navigation result", result
    
    # find robot position
    pose = almath.Pose2D(motion_service.getRobotPosition(False))
    print "Pose", pose.x, pose.y, pose.theta

    
 
    # Go to rest position
    #time.sleep(1)
    #motion_service.rest()

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.2.100", help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
