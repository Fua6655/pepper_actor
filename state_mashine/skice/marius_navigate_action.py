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
"""
roslaunch pepper_interaction pepper_robot.launch   
rosrun map_server map_saver -f mapa
rosrun map_server map_server mapa.yaml
roslaunch pepper_interaction plymouth_move_base.launch
rosrun pepper_interaction basic_awareness_server.py
rosrun smach_viewer smach_viewer.py
rosrun pepper_interaction marius.py
"""

def main():

    rospy.init_node('Marius_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('wake_up', ServiceState('/pepper_robot/pose/wakeup', Empty),
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Navigate robot to WayPoint actionlib
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = 1.0
        move_base_goal.target_pose.pose.position.y = 0.0
        move_base_goal.target_pose.pose.orientation.z = 0.0
        move_base_goal.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('navigate_to_point', SimpleActionState('move_base',MoveBaseAction, goal=move_base_goal),
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        #### Basic awareness preset ###
        # Set the service for EngagementMode                         "Unengaged" "SemiEngaged" "FullyEngaged"
        smach.StateMachine.add('set_EngagementMode', ServiceState('/setEngagementMode', SetString, "Unengaged"),
                                transitions={'succeeded':'set_TrackingMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
        

        # Set the service for TrackingMode                        "Head" "BodyRotation" "WholeBody" "MoveContextually"
        smach.StateMachine.add('set_TrackingMode', ServiceState('/setTrackingMode', SetString, "BodyRotation"),
                                transitions={'succeeded':'enable_awareness',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
        


        # Starting Basic awareness                                                     True False
        smach.StateMachine.add('enable_awareness', ServiceState('/enableBasicAwareness', SetBool, True),
                                transitions={'succeeded':'navigate_to_human',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


        # Navigate to human
        def human_goal_cb(userdata, goal):

            sub = rospy.Subscriber('/detected_human_pose', PoseStamped, dataPose) 
            rate = rospy.Rate(10) # 10hz
            return move_base_goal

        def dataPose(data):
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = data.header.frame_id
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_goal.target_pose.pose.position.x = data.pose.position.x
            move_base_goal.target_pose.pose.position.y = data.pose.position.y
            move_base_goal.target_pose.pose.orientation.z = data.pose.orientation.z
            move_base_goal.target_pose.pose.orientation.w = data.pose.orientation.w # 1.0


        smach.StateMachine.add('navigate_to_human',
                         SimpleActionState('move_base',
                                           MoveBaseAction,
                                           goal_cb=human_goal_cb),
                         transitions={'succeeded':'disable_awareness',
                                      'aborted':'aborted',
                                      'preempted':'preempted'},
                         remapping={'move_base_goal_input':'userdata_input'})
        # prvi imput je za akciju, drugi za state mashine





        # Stop Basic awareness                                                     True False
        smach.StateMachine.add('disable_awareness', ServiceState('/enableBasicAwareness', SetBool, False),
                                transitions={'succeeded':'rest',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Go to rest pose
        smach.StateMachine.add('rest', ServiceState('/pepper_robot/pose/rest', Empty),
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
