#!/usr/bin/env python

import rospy
import smach
#import smach_ros
from std_srvs.srv import Empty
from smach_ros import *
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PickupActionGoal
import time
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

from naoqi_bridge_msgs.msg import BodyPoseWithSpeedActionGoal, JointTrajectoryActionGoal, SetSpeechVocabularyActionGoal, RunBehaviorActionGoal
"""
roslaunch pepper_interaction pepper.launch 

roslaunch pepper_interaction servers.launch 

rosrun pepper_interaction sulla_scena1.py
"""
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['time'])
    def execute(self, userdata):
        time.sleep(userdata.time) 
        return 'succeeded'


class GoToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['pose', 'time'])
        self.pose_pub = rospy.Publisher('/body_pose_naoqi/goal', BodyPoseWithSpeedActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        pose_goal = BodyPoseWithSpeedActionGoal()
        pose_goal.goal.posture_name = userdata.pose
        pose_goal.goal.speed = 0.5
        self.pose_pub.publish(pose_goal)
        time.sleep(userdata.time) 
        return 'succeeded'

class Fill_Vocabulary(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['vocabulary_data'])
        self.vocabulary_pub = rospy.Publisher('/speech_vocabulary_action/goal', SetSpeechVocabularyActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        vocalubary_goal = SetSpeechVocabularyActionGoal()
        vocalubary_goal.goal.words = userdata.vocabulary_data
        self.vocabulary_pub.publish(vocalubary_goal)
        return 'succeeded'


class Run_Behavior(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['behavior', 'time'])
        self.behavior_pub = rospy.Publisher('/run_behavior/goal', RunBehaviorActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        behavior_goal = RunBehaviorActionGoal()
        behavior_goal.goal.behavior = userdata.behavior
        self.behavior_pub.publish(behavior_goal)
        time.sleep(userdata.time) 
        return 'succeeded'



class JointTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['joint_names', 'joint_points', 'time'])
        self.joint_pub = rospy.Publisher('/joint_trajectory/goal', JointTrajectoryActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        joints_goal = JointTrajectoryActionGoal()
        #joints_goal.header.frame_id = "torso"
        #joints_goal.header.stamp = rospy.Time.now()
        joints_goal.goal.trajectory.joint_names = "RShoulderPitch" # userdata.joint_names
        joints_goal.goal.trajectory.points = [0.5, 0.5, 0.5, 0.5] #userdata.joint_points
        #joints_goal.goal.trajectory.points.positions = 0.5 
        #joints_goal.goal.trajectory.points.velocities = 0.5
        #joints_goal.goal.trajectory.points.accelerations = 0.5
        #joints_goal.goal.trajectory.points.effort = 0.5
        self.joint_pub.publish(joints_goal)
        time.sleep(userdata.time) 
        return 'succeeded'


def main():

    rospy.init_node('Stateovi')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    sm.userdata.time = 3.0
    sm.userdata.body_pose = "Stand"
    sm.userdata.vocabulary_data = 'yes, please, no, love, kill'
    sm.userdata.behavior1 = 'sleep-reactions/Animations/Start_ShakeHead'
    sm.userdata.behavior2 = 'sleep-reactions/Animations/End_ShakeHead'
    sm.userdata.joint_names = ''
    sm.userdata.joint_points = [0]

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('wake_up', ServiceState('/wakeup', Empty),
                                transitions={'succeeded':'run_behavior1',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

################################ ASR TTS #########################################

        # Say word via tts service                                                    
        smach.StateMachine.add('tts_1', ServiceState('/tts_speech', SetString, 'Yes'),
                                transitions={'succeeded':'pose_StandZero',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Say word via tts service  and animate while talking                                                   
        smach.StateMachine.add('tts_2', ServiceState('/tts_animation', SetString, 'your order for fifteen thousand robots'),
                                transitions={'succeeded':'vocabulary',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Fill in the vocabulary
        sm.userdata.vocabulary_data = 'yes, please, no, love, kill'
        smach.StateMachine.add('vocabulary', Fill_Vocabulary(),
                                transitions={'succeeded':'asr_start',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'vocabulary_data':'vocabulary_data'})

        # start asr                                 
        smach.StateMachine.add('asr_start', ServiceState('/start_asr', SetBool, True),
                                transitions={'succeeded':'wait_asr',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
        #Got detected word

        # Wait for some time
        smach.StateMachine.add('wait_asr', Wait(),
                                transitions={'succeeded':'asr_stop',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})

        # stop asr
        smach.StateMachine.add('asr_stop', ServiceState('/start_asr', SetBool, False),
                                transitions={'succeeded':'run_behavior1',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


######################################### Behaviours  ##############################

        # Run Behaviour
        smach.StateMachine.add('run_behavior1', Run_Behavior(),
                                transitions={'succeeded':'run_behavior2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior1'})


        # Run Behaviour
        smach.StateMachine.add('run_behavior2', Run_Behavior(),
                                transitions={'succeeded':'pose_StandZero',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior2'})



######################################### Pose  ##############################

        # Go to pose Stand
        sm.userdata.body_pose = "Stand"
        smach.StateMachine.add('pose_StandZero', GoToPose(),
                                transitions={'succeeded':'joint_trajectory',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'pose':'body_pose'})


        # Joint trajectory

        # 'HeadYaw, HeadPitch'
        # 'LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand'
        # 'RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw, RHand'
        # 'HipRoll, HipPitch, KneePitch'

        sm.userdata.joint_names = 'LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand'
        sm.userdata.joint_points = [0.5, 0.5, 0.5, 0.5] 
        smach.StateMachine.add('joint_trajectory', JointTrajectory(),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'joint_names':'joint_names',
                                            'joint_points':'joint_points',
                                             'time': 'time'})





        # Go to rest pose
        smach.StateMachine.add('rest', ServiceState('/rest', Empty),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

    #########################################################################
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/Sulla')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
