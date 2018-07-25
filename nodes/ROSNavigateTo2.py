#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped

rospy.init_node("moveit_visualization_sample")

# publisher to change the arm group
selector = rospy.Publisher("/rviz/moveit/select_planning_group", String)
plan = rospy.Publisher("/rviz/moveit/plan", Empty)
execute = rospy.Publisher("/rviz/moveit/execute", Empty)
reset_goal_state = rospy.Publisher("/rviz/moveit/update_goal_state", Empty)

# single arm 
rospy.sleep(5)
rospy.loginfo("move right_arm")
reset_goal_state.publish(Empty())
selector.publish(String("right_arm"))
arm_pub = rospy.Publisher("/rviz/moveit/move_marker/goal_r_wrist_roll_link", 
                          PoseStamped)
rospy.sleep(2)

goal = PoseStamped()
goal.header.frame_id = "/base_link"
goal.header.stamp = rospy.Time.now()
goal.pose.position.z = 1.1
goal.pose.position.x = 0.7
goal.pose.position.y = -0.3
goal.pose.orientation.w = 1.0
arm_pub.publish(goal)
rospy.sleep(1)
plan.publish(Empty())

rospy.sleep(5)

# dual arm
rospy.loginfo("move arms")
reset_goal_state.publish(Empty())
selector.publish(String("arms"))
larm_pub = rospy.Publisher("/rviz/moveit/move_marker/goal_l_wrist_roll_link", 
                           PoseStamped)
rarm_pub = rospy.Publisher("/rviz/moveit/move_marker/goal_r_wrist_roll_link", 
                           PoseStamped)
rospy.sleep(2)

rgoal = PoseStamped()
rgoal.header.frame_id = "/base_link"
rgoal.header.stamp = rospy.Time.now()
rgoal.pose.position.z = 1.1
rgoal.pose.position.x = 0.7
rgoal.pose.position.y = -0.3
rgoal.pose.orientation.w = 1.0
rarm_pub.publish(rgoal)

lgoal = PoseStamped()
lgoal.header.frame_id = "/base_link"
lgoal.header.stamp = rospy.Time.now()
lgoal.pose.position.z = 1.1
lgoal.pose.position.x = 0.7
lgoal.pose.position.y = 0.3
lgoal.pose.orientation.w = 1.0
larm_pub.publish(lgoal)

rospy.sleep(1)
plan.publish(Empty())
rospy.sleep(5)

# base
rospy.loginfo("move base")
reset_goal_state.publish(Empty())
selector.publish(String("base"))

base_pub = rospy.Publisher("/rviz/moveit/move_marker/goal_base_footprint", 
                           PoseStamped)
rospy.sleep(2)
goal = PoseStamped()
goal.header.frame_id = "/base_link"
goal.header.stamp = rospy.Time.now()
goal.pose.position.z = 0
goal.pose.position.x = 0.5
goal.pose.position.y = 0
goal.pose.orientation.w = 1.0
base_pub.publish(goal)
rospy.sleep(2)
plan.publish(Empty())
rospy.sleep(5)
