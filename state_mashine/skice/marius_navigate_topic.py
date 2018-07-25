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
roslaunch pepper_interaction pepper_smach.launch   
rosrun pepper_interaction basic_awareness_server_param.py 
rosrun pepper_interaction marius_navigate_topic.py
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
        while i <5:
            self.base_pub.publish(goal)
            #print goal
            i = i+1
        return 'succeeded'


class Get_human_distance(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    output_keys=['x','y','z','w'])
        self.xx = 0
        self.yy = 0
        self.zz = 0
        self.ww = 0
        sub = rospy.Subscriber('/detected_human_pose', PoseStamped, self.dataPose) 

    def execute(self, userdata):
        detected = rospy.get_param('human_distance')
        while detected == 'False':
            try:
                detected = rospy.get_param('human_distance')    
                print detected 
                #rate = rospy.Rate(10) # 10hz
            except KeyError:
                print "value not set"
        userdata.x = self.xx
        userdata.y = self.yy
        userdata.z = self.zz
        userdata.w = self.ww


    def dataPose(data):
        self.xx = data.pose.position.x
        self.yy = data.pose.position.y
        self.zz = 0.0 #data.pose.orientation.z
        self.ww = 1.0 #data.pose.orientation.w # 1.0
        try:
            rospy.set_param('human_distance', 'False')
        except KeyError:
            print "value not set"
        return 'succeeded'


def main():

    rospy.init_node('Marius_smach_topic')

    # Create a SMACH state machinen
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Create user data
    sm.userdata.pose_x = 0.2
    sm.userdata.pose_y = 0.2
    sm.userdata.pose_z = 0.0
    sm.userdata.pose_w = 1.0
 
    
    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('wake_up', ServiceState('/pepper_robot/pose/wakeup', Empty),
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

        # Navigate robot to WayPoint topic
        smach.StateMachine.add('navigate_to_point', Move_base_simple(),
                                transitions={'succeeded':'set_EngagementMode',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'x':'pose_x',
                                           'y':'pose_y',
                                           'z':'pose_z',
                                           'w':'pose_w',})



        #### Basic awareness preeset ###
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
                                transitions={'succeeded':'get_human_pose',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


        # Get human pose
        smach.StateMachine.add('get_human_pose', Get_human_distance(),
                                transitions={'succeeded':'navigate_to_human',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'x':'pose_x',
                                           'y':'pose_y',
                                           'z':'pose_z',
                                           'w':'pose_w',})

        # Navigate robot to human topic
        smach.StateMachine.add('navigate_to_human', Move_base_simple(),
                                transitions={'succeeded':'disable_awareness',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'x':'pose_x',
                                           'y':'pose_y',
                                           'z':'pose_z',
                                           'w':'pose_w',})



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
