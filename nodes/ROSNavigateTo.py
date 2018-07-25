#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

Twist_msg = Twist()
Odometry_msg = Odometry()

def dataTwist(data):
    try:
        Twist_msg.linear.x = data.linear.x
        Twist_msg.linear.y = data.linear.y
        Twist_msg.angular.z = data.angular.z
    except:
        rospy.loginfo("error happens in data Twist")

def dataOdometry(data):
    try:
        Odometry_msg.twist.twist.linear.x = data.linear.x
        Odometry_msg.twist.twist.linear.y = data.linear.y
        Odometry_msg.twist.twist.angular.z = data.angular.z
        rospy.loginfo("%s ",Odometry_msg)
    except:
        rospy.loginfo("error happens in data Odometry")
    

def move_base():
    base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    #vel_sub = rospy.Subscriber('/cmd_vel', Twist, dataTwist) #must run before pepper_move_base.launch 
    odom_sub = rospy.Subscriber('/pepper_robot/odom', Odometry, dataOdometry) 
    rospy.init_node('NavigateTo', anonymous=True)
    rate = rospy.Rate(10) # 10hz
  
    rospy.loginfo("move base")

    goal = PoseStamped()
    goal.header.frame_id = "torso" # "base_footprint"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = 0.2  # = input("Type position.x:")
    goal.pose.position.y = 0.2
    goal.pose.position.z = 0.2
    goal.pose.orientation.z = 0.1
    goal.pose.orientation.w = 0.1

    #rospy.loginfo("%s ",Twist_msg)

    while (Odometry_msg.twist.twist.linear.x==0) or (Odometry_msg.twist.twist.linear.y==0): #/pepper_robot/odom 
        base_pub.publish(goal)
        rospy.loginfo("%s ",Odometry_msg)
        rospy.loginfo("%s", goal)
        rospy.sleep(2)
      
    rate.sleep()

if __name__ == '__main__':
    try:
        move_base()
    except rospy.ROSInterruptException:
        pass



