#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use moveToward Method"""

import qi
import argparse
import sys
import time


def main(session):
    motionProxy  = session.service("ALMotion")
    localizationProxy = session.service("ALLocalization")

    # Learning home.
    ret = localizationProxy.learnHome()
    # Check that no problem occurred.
    if ret == 0:
        print "Learning OK"
    else:
        print "Error during learning " + str(ret)

    # Make some moves.
    motionProxy.moveTo(0.5, 0.0, 0.2)

    # Go back home.
    ret = localizationProxy.goToHome()
    # Check that no problem occurred.
    if ret == 0:
        print "go to home OK"
    else:
        print "error during go to home " + str(ret)

    # Save the data for later use.
    ret = localizationProxy.save("example")
    # Check that no problem occurred.
    if ret == 0:
        print "saving OK"
    else:
        print "error during saving" + str(ret)




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.2.100",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
