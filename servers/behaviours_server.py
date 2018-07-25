#!/usr/bin/env python

import time
import threading
import rospy
import actionlib
from naoqi import ALProxy
from naoqi_driver.naoqi_node import NaoqiNode

from naoqi_bridge_msgs.msg import RunBehaviorAction

from naoqi_bridge_msgs.srv import (
    GetInstalledBehaviors,
    GetInstalledBehaviorsResponse,
    )

class NaoBehaviors(NaoqiNode):
    #This should be treated as a constant
    NODE_NAME = "nao_behaviors"

    def __init__( self ):

        #Initialisation
        NaoqiNode.__init__( self, self.NODE_NAME )

        #We need this variable to be able to call stop behavior when preempted
        self.behavior = None
        self.lock = threading.RLock()

        #Proxy for listingBehaviors and stopping them

        #IP = "192.168.1.111"
        #PORT = 9559
        ip = rospy.get_param('/behaviours_server/pepper_ip')
        port = rospy.get_param('/behaviours_server/pepper_port')

        self.behaviorProxy = ALProxy("ALBehaviorManager", ip, port)

        # Register ROS services
        self.getInstalledBehaviorsService = rospy.Service(
            "get_installed_behaviors",
            GetInstalledBehaviors,
            self.getInstalledBehaviors
            )

        #Prepare and start actionlib server
        self.actionlibServer = actionlib.SimpleActionServer(
            "run_behavior",
            RunBehaviorAction,
            self.runBehavior,
            False
            )

        #self.actionlibServer.register_preempt_callback( self.stopBehavior )

        self.actionlibServer.start()

    def getInstalledBehaviors( self, request ):
        result = self.behaviorProxy.getInstalledBehaviors()
        return GetInstalledBehaviorsResponse( result )


    def runBehavior( self, request ):
        #Note this function is executed from a different thread
        rospy.logdebug(
            "Execution of behavior: '{}' requested".format(request.behavior))

        #Check requested behavior is installed
        if not request.behavior in self.behaviorProxy.getInstalledBehaviors():
            error_msg = "Behavior '{}' not installed".format(request.behavior)
            self.actionlibServer.set_aborted(text = error_msg)
            rospy.logdebug(error_msg)
            return

        with self.lock:
            # Check first if we're already preempted, and return if so
            if self.actionlibServer.is_preempt_requested():
                self.actionlibServer.set_preempted()
                rospy.logdebug("Behavior execution preempted before it started")
                return

            #Save name of behavior to be run
            self.behavior = request.behavior
            #Execute behavior (on another thread so we can release lock)
            taskID = self.behaviorProxy.post.runBehavior( self.behavior )

        # Wait for task to complete (or be preempted)
        rospy.logdebug("Waiting for behavior execution to complete")
        self.behaviorProxy.wait( taskID, 0 )
        time.sleep(2)
        #Evaluate results
        with self.lock:
            self.behavior = None
            # If preempted, report so
            if self.actionlibServer.is_preempt_requested() :
                self.actionlibServer.set_preempted()
                rospy.logdebug("Behavior execution preempted")
            # Otherwise, set as succeeded
            else:
                self.actionlibServer.set_succeeded()
                rospy.logdebug("Behavior execution succeeded")

    def stopBehavior( self ):
        with self.lock:
            if self.behavior and self.actionlibServer.is_active() :
                self.behaviorProxy.stopBehavior( self.behavior )


if __name__ == '__main__':
    node = NaoBehaviors()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)

