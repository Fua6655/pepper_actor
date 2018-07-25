#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import argparse
import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.srv import (
    SetFloatResponse,
    SetFloat,
    SetStringResponse,
    SetString,
    GetFloatResponse,
    GetFloat)
from std_srvs.srv import (
    SetBool,
    SetBoolResponse)

class NaoqiBasicAwareness (NaoqiNode):
    def __init__(self, human_tracked_event_watcher_session):
        NaoqiNode.__init__(self, 'naoqi_awareness')
        #sad ide poziv druge klase
        
        self.proxy = human_tracked_event_watcher_session
        self.setEngagementSrv = rospy.Service("setEngagementMode", SetString, self.handleSetEngagementMode)
        self.setTrackingSrv = rospy.Service("setTrackingMode", SetString, self.handleSetTrackingMode)
        self.setTrackingSrv = rospy.Service("enableBasicAwareness", SetBool, self.handleEnableBasicAwareness)
        #self.getHumanSrv = rospy.Service("getHumanDistance", GetFloat, self.handleGetHumanDistance)
        rospy.loginfo("naoqi_basic_awareness is initialized")

    def handleSetEngagementMode(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setEngagementMode(req.data) #   "Unengaged" "SemiEngaged" "FullyEngaged"
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTrackingMode(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setTrackingMode(req.data) # "Head" "BodyRotation" "WholeBody" "MoveContextually"
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleEnableBasicAwareness(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setEnabled(req.data) #True of False
            res.success = True
            res.message = str(req.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
   

def connectQi(pip, pport):
    rospy.loginfo("Connecting to qi at %s:%d", pip, pport)

    # Initialize qi framework.
    connection_url = "tcp://" + pip + ":" + str(pport)
    app = qi.Application(["HumanTrackedEventWatcher", "--qi-url=" + connection_url])
    if app is None:
         rospy.logerr("Could not connect to qi fremework")
         exit(1)
    else:
        return app


class HumanTrackedEventWatcher(object):
    """ A class to react to HumanTracked and PeopleLeft events """

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(HumanTrackedEventWatcher, self).__init__()

        try:
            app.start()
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " +
                   str(args.port) + ".\n")

            sys.exit(1)

        session = app.session
        self.subscribers_list = []
        self.is_speech_reco_started = False

        self.memory = session.service("ALMemory")
        self.speech_reco = session.service("ALSpeechRecognition")
        self.basic_awareness = session.service("ALBasicAwareness")
        #self.motion = session.service("ALMotion")
        self.connect_callback("ALBasicAwareness/HumanTracked", self.on_human_tracked)
        self.connect_callback("ALBasicAwareness/HumanLost", self.on_people_left)

        #self.pub = rospy.Publisher('/detected_human_pose', PoseStamped, queue_size=10) mora prvo inicijalizirati ros node
        #self.goal_test = PoseStamped()
        #self.pub.publish(self.goal_test)

    def connect_callback(self, event_name, callback_func):
        """ connect a callback for a given event """
        subscriber = self.memory.subscriber(event_name)
        subscriber.signal.connect(callback_func)
        self.subscribers_list.append(subscriber)

    def on_human_tracked(self, value):
        """ callback for event HumanTracked """
        print "Got HumanTracked: detected person with ID:", str(value)
        if value >= 0:  # found a new person

            self.pub = rospy.Publisher('/detected_human_pose', PoseStamped, queue_size=10)
            #self.start_speech_reco()
            position_human = self.get_people_perception_data(value)
            [x, y, z] = position_human
            print "The tracked person with ID", value, "is at the position:", \
                "x=", x, "/ y=",  y, "/ z=", z
            self.goal = PoseStamped()
            self.goal.header.frame_id = "torso" # "base_footprint" PositionInWorldFrame
            self.goal.header.stamp = rospy.Time.now()
            self.goal.pose.position.x = x 
            self.goal.pose.position.y = y
            self.goal.pose.position.z = z
            self.goal.pose.orientation.z = 0.
            self.goal.pose.orientation.w = 1.0

            i = 0
            self.pub.publish(self.goal)
            time.sleep(0.5)
            while i<10:
                rate = rospy.Rate(10) # 10hz
                self.pub.publish(self.goal)
                i = i+1
            rate.sleep()

    def on_people_left(self, value):
        """ callback for event PeopleLeft """
        print "Got PeopleLeft: lost person", str(value)
        #self.stop_speech_reco()

    def start_speech_reco(self):
        """ start ASR when someone's detected in event handler class """
        #vokabular se mora napuniti prije pokretanja klase
        if not self.is_speech_reco_started:
            try:
                self.speech_reco.setVocabulary(["yes", "no"], False) # napraviti servis u  NaoqiBasicAwareness za puniti vocabular
            except RuntimeError:
                print "ASR already started"

            print "Starting speech recognition"
            self.speech_reco.subscribe("BasicAwareness_Test")
            self.is_speech_reco_started = True

    def stop_speech_reco(self):
        """ stop ASR when someone's detected in event handler class """
        if self.is_speech_reco_started:
            print "Stopping speech recognition"
            self.speech_reco.unsubscribe("BasicAwareness_Test")
            self.is_speech_reco_started = False

    def get_people_perception_data(self, id_person_tracked):
        """
            return information related to the person who has the id
            "id_person_tracked" from People Perception
        """
        memory_key = "PeoplePerception/Person/" + str(id_person_tracked) + \
                     "/PositionInRobotFrame"  #Torso world
        return self.memory.getData(memory_key)

    def run(self):
        """
            ova metoda se nekoristi
        """
        # start
        print "Waiting for the robot to be in wake up position"
        self.motion.wakeUp()

        print "Starting BasicAwareness with the fully engaged mode"
        self.basic_awareness.setEngagementMode("FullyEngaged")
        self.basic_awareness.setTrackingMode("MoveContextually")
        self.basic_awareness.setEnabled(True)

        # loop on, wait for events until manual interruption
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, shutting down"
            # stop
            print "Stopping BasicAwareness"
            self.basic_awareness.setEnabled(False)

            self.stop_speech_reco()

            print "Waiting for the robot to be in rest position"
            self.motion.rest()

            sys.exit(0)


if __name__ == "__main__":

    #parser = argparse.ArgumentParser()
    #parser.add_argument("--ip", type=str, default="192.168.1.111", help="Robot IP address")
    #parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")

    #args = parser.parse_args()
    #print (" %s",args.ip)

    #app=connectQi(args.ip, args.port)


    pepper_ip = rospy.get_param('/basic_awareness_server/pepper_ip')
    pepper_port = rospy.get_param('/basic_awareness_server/pepper_port')

    app=connectQi(pepper_ip, pepper_port)
    
    
    human_tracked_event_watcher = HumanTrackedEventWatcher(app)

    naoqi_basic_awareness = NaoqiBasicAwareness(human_tracked_event_watcher)


  
    #human_tracked_event_watcher.run()

    rospy.spin()

