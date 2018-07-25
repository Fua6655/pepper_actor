#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import argparse
import sys
import time
import rospy
import actionlib
from std_msgs.msg import String
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
from dynamic_reconfigure.server import Server as ReConfServer
import dynamic_reconfigure.client
from std_srvs.srv import( Empty, EmptyResponse )
from naoqi_bridge_msgs.msg import(
    WordRecognized,
    SetSpeechVocabularyGoal,
    SetSpeechVocabularyResult,
    SetSpeechVocabularyAction,
    SpeechWithFeedbackGoal,
    SpeechWithFeedbackResult,
    SpeechWithFeedbackFeedback,
    SpeechWithFeedbackAction )


class ROS_ASR_TTS (NaoqiNode):
    def __init__(self, event_watcher_session):
        NaoqiNode.__init__(self, 'naoqi_asr_tts')
        self.is_vocabulary_filled = False
        
        self.proxy = event_watcher_session
        # Servisi
        #self.setVocabularySrv = rospy.Service("set_vocabulary", SetString, self.handleSetVocabulary) #koristi se akcija, njen goal
        self.TTSSpeechSrv = rospy.Service("tts_speech", SetString, self.handleTTSSpeech)
        self.TTSAnimationSrv = rospy.Service("tts_animation", SetString, self.handleTTSAnimation)
           
        self.startASRSrv = rospy.Service("start_asr", SetBool, self.handleStartASR)
        #self.setLanguageSrv = rospy.Service("set_language", SetString, self.handleSetLanguage) ima vec servis /naoqi_driver/set_language
        rospy.loginfo("naoqi_asr_tts is initialized")

        #Akcije
        # Actionlib server for altering the speech recognition vocabulary
        self.setSpeechVocabularyServer = actionlib.SimpleActionServer("speech_vocabulary_action", SetSpeechVocabularyAction,
                                                                  execute_cb=self.executeSpeechVocabularyAction,
                                                                  auto_start=False)

        # Actionlib server for having speech with feedback
        self.speechWithFeedbackServer = actionlib.SimpleActionServer("speech_action", SpeechWithFeedbackAction,
                                                                  execute_cb=self.executeSpeechWithFeedbackAction,
                                                                  auto_start=False)
        # Start both actionlib servers
        self.setSpeechVocabularyServer.start()
        self.speechWithFeedbackServer.start() # koristi se servis umjesto akcije



    def executeSpeechWithFeedbackAction(self, goal):
        # Gets the goal and begins the speech
        self.speech_with_feedback_flag = True
        saystr = goal.say
        self.internalSay(saystr)

        # Wait till the onTextDone event is called or 2 mins are passed
        counter = 0
        while self.speech_with_feedback_flag == True and counter < 1200:
            rospy.sleep(0.1)
            counter += 1

        # Send the success feedback
        self.speechWithFeedbackServer.set_succeeded()


    def executeSpeechVocabularyAction(self, goal):
        #~ Called by action client
        rospy.loginfo("SetSpeechVocabulary action executing");

        words = goal.words
        words_str = ""

        #~ Empty word list. Send failure.
        if len(words) == 0:
            setVocabularyResult = SetSpeechVocabularyResult()
            setVocabularyResult.success = False
            self.setSpeechVocabularyServer.set_succeeded(setVocabularyResult)
            return

        # Save the vocabulari, just this is importat for one words
        # setVocabulary(const std::vector<std::string>& vocabulary, const bool& enableWordSpotting)
        # wee nedd enabled word spoting for detecting word in sentence
        
        #~ Create the vocabulary string
        for i in range(0, len(words) - 1):
            words_str += str(words[i]) #+ "/"

        words_str += words[len(words) - 1]
        print words_str
        vocabulary =  words_str.split(", ")

        self.proxy.asr.setVocabulary(vocabulary, True)

        #~ Update the dynamic reconfigure vocabulary parameter
        params = { 'vocabulary' : words_str }
        #self.reconf_client.update_configuration(params)
        print params
        #~ Send success
        setVocabularyResult = SetSpeechVocabularyResult()
        setVocabularyResult.success = True
        self.setSpeechVocabularyServer.set_succeeded(setVocabularyResult)

 
    def handleSetVocabulary(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.asr.setLanguage("English")
            #vocabulary = ["yes", "no", "please"]
            print req.data
            self.proxy.asr.setVocabulary([req.data], False)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleTTSSpeech(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.tts.setParameter("speed", 85)
            self.proxy.tts.say(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res


    def handleTTSAnimation(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.tts.setParameter("speed", 85)
            # set the local configuration
            configuration = {"bodyLanguageMode":"contextual"} #“disabled”, “random”, “contextual”
            # say the text with the local configuration
            self.proxy.tts_animation.say(req.data, configuration)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleStartASR(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            if req.data == True:
                self.proxy.asr.subscribe("Test_ASR")
                res.success = True
                res.message = str(req.data)
                return res

            elif req.data == False:
                self.proxy.asr.unsubscribe("Test_ASR")
                res.success = True
                res.message = str(req.data)
                return res

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetLanguage(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.dialog.setLanguage(req.data) # ['Japanese', 'English', 'French']
            instaled_languages = self.proxy.dialog.getLanguage()
            print  instaled_languages
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res


def connectQi(pip, pport):
    rospy.loginfo("Connecting to qi at %s:%d", pip, pport)

    # Initialize qi framework.
    connection_url = "tcp://" + pip + ":" + str(pport)
    app = qi.Application(["SoundEventWatcher", "--qi-url=" + connection_url])
    if app is None:
         rospy.logerr("Could not connect to qi fremework")
         exit(1)
    else:
        return app


class SoundEventWatcher(object):
    """ A class to react to  """

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(SoundEventWatcher, self).__init__()

        try:
            app.start()
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " +
                   str(args.port) + ".\n")

            sys.exit(1)

        session = app.session
        self.subscribers_list = []

        self.memory = session.service("ALMemory")
        self.asr = session.service("ALSpeechRecognition")
        self.tts = session.service("ALTextToSpeech")
        self.tts_animation = session.service("ALAnimatedSpeech")
        self.dialog = session.service("ALDialog")
                
        self.connect_callback("WordRecognized", self.on_word_recognized)
        self.connect_callback("WordRecognizedAndGrammar", self.on_word_and_grammar_recognized)
        self.connect_callback("SpeechDetected", self.on_speech_detected)
        
        self.pub_word = rospy.Publisher('/detected_word', String, queue_size=10)
        self.pub_word_and_grammar = rospy.Publisher('/detected_word_and_grammar', String, queue_size=10)
        self.pub_speech = rospy.Publisher('/detected_speech', String, queue_size=10)

    def connect_callback(self, event_name, callback_func):
        """ connect a callback for a given event """
        subscriber = self.memory.subscriber(event_name)
        subscriber.signal.connect(callback_func)
        self.subscribers_list.append(subscriber)

    def on_word_recognized(self, value):
        print "Got recognized the word", str(value)

        if len(value) > 0:  # found a new word
            
            rate = rospy.Rate(10) # 10hz
            self.pub_word.publish(str(value))
            rate.sleep()

    def on_word_and_grammar_recognized(self, value):
        print "Got recognized the word and grammar", str(value)

        if len(value) > 0:  # found a new word

            rate = rospy.Rate(10) # 10hz
            self.pub_word_and_grammar.publish(str(value))
            rate.sleep()

    def on_speech_detected(self, value):
        print "Got recognized speech", str(value)

        if value > 0:  # found a new word
            
            rate = rospy.Rate(10) # 10hz
            self.pub_speech.publish(str(value))
            rate.sleep()


if __name__ == "__main__":

    #parser = argparse.ArgumentParser()
    #parser.add_argument("--ip", type=str, default="192.168.1.111", help="Robot IP address")
    #parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")

    #args = parser.parse_args()
    #app=connectQi(args.ip, args.port)
    pepper_ip = rospy.get_param('/asr_tts_server/pepper_ip')
    pepper_port = rospy.get_param('/asr_tts_server/pepper_port')

    app=connectQi(pepper_ip, pepper_port)
    
    naoqi_asr_tts_event_watcher = SoundEventWatcher(app)

    ros_asr_tts = ROS_ASR_TTS(naoqi_asr_tts_event_watcher)

    rospy.spin()
