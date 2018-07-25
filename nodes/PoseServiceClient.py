#!/usr/bin/env python


import rospy
from std_srvs.srv import Empty


def pose_server():
    rospy.init_node('pose_manager')
    rospy.wait_for_service('/pepper_robot/pose/wakeup')
    rospy.ServiceProxy('/pepper_robot/pose/wakeup', Empty,)

    print "WakeUp"
    rospy.spin()

if __name__ == "__main__":
    pose_server()



