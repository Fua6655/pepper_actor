#!/usr/bin/env python


import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    SetBoolResponse,
    SetBool,
    TriggerResponse,
    Trigger
)


class wakeUp(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'go_to_wake')
        self.connectNaoQi()
        #self.postureProxy.goToPosture("StandInit", 0.5)
        #self.postureProxy.goToPosture("Crouch", 0.5)
        
        self.motionProxy.wakeUp()

        rospy.loginfo("Pepper waked up")

        try:
            while True:
                rospy.sleep(1)
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user, shutting down")
            # stop
          
            rospy.loginfo("Waiting for the robot to be in rest position")
            self.motionProxy.rest()

        sys.exit(0)

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.systemProxy = self.get_proxy("ALSystem")
        if self.systemProxy is None:
            rospy.logerr("Could not get a proxy to ALSystem")
            exit(1)
        self.motionProxy  = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            rospy.logerr("Could not get a proxy to ALMotion")
            exit(1)
        self.postureProxy  = self.get_proxy("ALRobotPosture")
        if self.postureProxy is None:
            rospy.logerr("Could not get a proxy to ALRobotPosture")
            exit(1)
        self.basic_awarenessProxy  = self.get_proxy("ALBasicAwareness")
        if self.basic_awarenessProxy is None:
            rospy.logerr("Could not get a proxy to ALBasicAwareness")
            exit(1)
        self.memoryProxy  = self.get_proxy("ALMemory")
        if self.memoryProxy is None:
            rospy.logerr("Could not get a proxy to ALMemory")
            exit(1)
    
if __name__ == '__main__':
    wakeUp()
    rospy.spin()

