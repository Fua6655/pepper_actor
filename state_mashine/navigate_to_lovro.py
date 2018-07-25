#!/usr/bin/env python

import rospy
import smach
#import smach_ros
from std_srvs.srv import Empty
from smach_ros import *
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PickupActionGoal
from naoqi_bridge_msgs.msg import BodyPoseWithSpeedActionGoal, JointTrajectoryActionGoal, SetSpeechVocabularyActionGoal, RunBehaviorActionGoal

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
import time



"""
roslaunch pepper_interaction pepper.launch 

roslaunch pepper_interaction gmapping_pointCloud.launch 
# or
roslaunch pepper_interaction gmapping_laser.launch
# or
roslaunch pepper_interaction gmapping_octomap.launch

rosrun map_server map_saver -f /home/luka/catkin_ws/src/pepper_interaction/maps/map.yaml

roslaunch pepper_interaction navigation.launch
#or
roslaunch pepper_interaction navigation2.launch

roslaunch pepper_interaction servers.launch 

rosrun pepper_interaction navigate_to_human.py
"""

class Move_base_simple(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['x','y','z','w'])
        self.base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rate = rospy.Rate(10) # 10hz

    def execute(self, userdata):
        goal = PoseStamped()
        goal.header.frame_id = "torso" # "base_footprint"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = userdata.x 
        goal.pose.position.y = userdata.y
        goal.pose.orientation.z = userdata.z
        goal.pose.orientation.w = userdata.w
        i = 0
        self.base_pub.publish(goal)
        print goal
        while i <5:
            self.base_pub.publish(goal)
            print goal
            i = i+1
        return 'succeeded'


class Get_human_distance(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    output_keys=['x','y','z','w'])

        
    def execute(self, userdata):
        msg = rospy.wait_for_message('/detected_human_pose', PoseStamped)
        #sub = rospy.Subscriber('/detected_human_pose', PoseStamped, self.dataPose) 
        print msg
        userdata.x = msg.pose.position.x
        userdata.y = msg.pose.position.y
        userdata.z = 0.0
        userdata.w = 1.0

        return 'succeeded'

    def dataPose(data):
        self.xx = data.pose.position.x
        self.yy = data.pose.position.y
        self.zz = 0.0 #data.pose.orientation.z
        self.ww = 1.0 #data.pose.orientation.w # 1.0
        return 'succeeded'


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['time'])
       
    def execute(self, userdata):
        print userdata.time
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
    #TODO: Omoguciti mogucnost manipulacije i mogucnost postavljanja stiffnesa
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

    rospy.init_node('Navigate_to_human')

    # Create a SMACH state machinen
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Create user data
    sm.userdata.pose_x = 0.2
    sm.userdata.pose_y = 0.2
    sm.userdata.pose_z = 0.0
    sm.userdata.pose_w = 1.0
    sm.userdata.time = 3.0
    
    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('wake_up', ServiceState('/wakeup', Empty),
                                transitions={'succeeded':'tts_2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        

        # Say word via tts service  and animate while talking                                                   
        smach.StateMachine.add('tts_2', ServiceState('/tts_animation', SetString, 'Hello I am robot Pepper. Now I can be programed via ROS so my abilities are bigger. I can detect human if he makes a sound or movement.'),
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})




        #### Basic awareness preeset ###
        # Set the service for EngagementMode                         "Unengaged" "SemiEngaged" "FullyEngaged"
        smach.StateMachine.add('set_EngagementMode', ServiceState('/setEngagementMode', SetString, "Unengaged"),
                                transitions={'succeeded':'set_TrackingMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
        

        # Set the service for TrackingMode                        "Head" "BodyRotation" "WholeBody" "MoveContextually"
        smach.StateMachine.add('set_TrackingMode', ServiceState('/setTrackingMode', SetString, "Head"),
                                transitions={'succeeded':'enable_awareness',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        # Starting Basic awareness                                                     True False
        smach.StateMachine.add('enable_awareness', ServiceState('/enableBasicAwareness', SetBool, True),
                                transitions={'succeeded':'get_human_pose',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


        # Get human pose
        smach.StateMachine.add('get_human_pose', Get_human_distance(),
                                transitions={'succeeded':'disable_awareness',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'x':'pose_x',
                                           'y':'pose_y',
                                           'z':'pose_z',
                                           'w':'pose_w',})

        # Stop Basic awareness                                                     True False
        smach.StateMachine.add('disable_awareness', ServiceState('/enableBasicAwareness', SetBool, False),
                                transitions={'succeeded':'tts_3',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


        # Say word via tts service  and animate while talking                                                   
        smach.StateMachine.add('tts_3', ServiceState('/tts_animation', SetString, 'Now I have detected human, so i can navigate to him'),
                                transitions={'succeeded':'wait_before_navigation',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        # Wait for some time
        smach.StateMachine.add('wait_before_navigation', Wait(),
                                transitions={'succeeded':'navigate_to_human_topic',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})


        # Navigate robot to human topic
        smach.StateMachine.add('navigate_to_human_topic', Move_base_simple(),
                                transitions={'succeeded':'wait_after_navigation',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'x':'pose_x',
                                           'y':'pose_y',
                                           'z':'pose_z',
                                           'w':'pose_w',})

      

        # Wait for some time
        smach.StateMachine.add('wait_after_navigation', Wait(),
                                transitions={'succeeded':'set_TrackingMode_2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})

        # Set the service for TrackingMode                        "Head" "BodyRotation" "WholeBody" "MoveContextually"
        smach.StateMachine.add('set_TrackingMode_2', ServiceState('/setTrackingMode', SetString, "MoveContextually"),
                                transitions={'succeeded':'enable_awareness2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


        # Starting Basic awareness                                                     True False
        smach.StateMachine.add('enable_awareness2', ServiceState('/enableBasicAwareness', SetBool, True),
                                transitions={'succeeded':'tts_4',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})




        # Say word via tts service  and animate while talking                                                   
        smach.StateMachine.add('tts_4', ServiceState('/tts_animation', SetString, 'I can read your emotion from your face or by voice. Would you like me to demonstrate you behaviours that I have for each of six basic emotions. You just need to say emotion!'),
                                transitions={'succeeded':'wait_after_navigation_2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
        #sad ovdje ide grananje na emocije ovisno o onome sto asr posalje

        # Wait for some time
        sm.userdata.time = 1.5
        smach.StateMachine.add('wait_after_navigation_2', Wait(),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})


        # Run Behaviour
        sm.userdata.behavior = 'animations/Stand/Emotions/Positive/Happy_2'
        smach.StateMachine.add('run_behavior', Run_Behavior(),
                                transitions={'succeeded':'disable_awareness2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior'})




        # Starting Basic awareness                                                     True False
        smach.StateMachine.add('disable_awareness2', ServiceState('/enableBasicAwareness', SetBool, False),
                                transitions={'succeeded':'rest',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})





        # Go to rest pose
        smach.StateMachine.add('rest', ServiceState('/rest', Empty),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


   
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/Pepper_Marius')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
