#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

MoveGroupActionGoal_msg = MoveGroupActionGoal()
JointAnglesWithSpeed_msg = JointAnglesWithSpeed()



def dataMoveGoal(data): #MoveGroupActionGoal_msg.goal.request.goal_constraints
    try:
        #JointAnglesWithSpeed_msg.joint_names = data.goal.request.goal_constraints
        #JointAnglesWithSpeed_msg.joint_angles = data.goal.request.goal_constraints
        JointAnglesWithSpeed_msg.joint_names = ["HeadYaw", "HeadPitch"]
        JointAnglesWithSpeed_msg.joint_angles = [0.1, 0.1]
        JointAnglesWithSpeed_msg.speed = 0.3
        JointAnglesWithSpeed_msg.relative = 0
    
        # sad se tu treba igrati sa stringovima. napraviti petljicu
        for i in data.goal.request.goal_constraints:
            rospy.loginfo("%s ",i)
           
        rospy.loginfo("%s ",data.goal.request.goal_constraints)
    except:
        rospy.loginfo("error happens in data MoveGoal")
        rospy.loginfo("%s ",data.goal.request.goal_constraints)
        for i in data.goal.request.goal_constraints:
            rospy.loginfo("%s ",i)
           

def move_group():
    joint_angles_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=10)
    moveit_group_goal_sub = rospy.Subscriber('/move_group/goal', MoveGroupActionGoal, dataMoveGoal) 
    rospy.init_node('MoveGroup', anonymous=True)
    rate = rospy.Rate(10) # 10hz
  
    rospy.loginfo("move group")

    while(1):
        JointAnglesWithSpeed_msg.header.frame_id = "torso" # "base_footprint"
        JointAnglesWithSpeed_msg.header.stamp = rospy.Time.now()
    
        joint_angles_pub.publish(JointAnglesWithSpeed_msg)


        #rospy.loginfo("%s ",JointAnglesWithSpeed_msg)
        rospy.sleep(2)
    rate.sleep()

if __name__ == '__main__':
    try:
        move_group()
    except rospy.ROSInterruptException:
        pass



