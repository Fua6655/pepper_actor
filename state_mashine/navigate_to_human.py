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
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Navigate robot to WayPoint topic
        smach.StateMachine.add('navigate_to_point_topic', Move_base_simple(),
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'x':'pose_x',
                                           'y':'pose_y',
                                           'z':'pose_z',
                                           'w':'pose_w',})

        # Navigate robot to WayPoint actionlib
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = sm.userdata.pose_x
        move_base_goal.target_pose.pose.position.y = sm.userdata.pose_y
        move_base_goal.target_pose.pose.orientation.z = sm.userdata.pose_z
        move_base_goal.target_pose.pose.orientation.w = sm.userdata.pose_w
        smach.StateMachine.add('navigate_to_point_action', SimpleActionState('move_base',MoveBaseAction, goal=move_base_goal),
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
                                transitions={'succeeded':'wait_before_navigation',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Wait for some time
        smach.StateMachine.add('wait_before_navigation', Wait(),
                                transitions={'succeeded':'navigate_to_human_action',
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

        # Navigate robot to human action
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = sm.userdata.pose_x
        move_base_goal.target_pose.pose.position.y = sm.userdata.pose_y
        move_base_goal.target_pose.pose.orientation.z = 0.0
        move_base_goal.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('navigate_to_human_action', SimpleActionState('move_base',MoveBaseAction, goal=move_base_goal),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        # Wait for some time
        smach.StateMachine.add('wait_after_navigation', Wait(),
                                transitions={'succeeded':'set_TrackingMode_2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})

        # Set the service for TrackingMode                        "Head" "BodyRotation" "WholeBody" "MoveContextually"
        smach.StateMachine.add('set_TrackingMode_2', ServiceState('/setTrackingMode', SetString, "MoveContextually"),
                                transitions={'succeeded':'succeeded',
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
