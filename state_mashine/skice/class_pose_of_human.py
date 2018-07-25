class PositionOfHuman(smach.State):     # igranje s msgs konverzija iz jednog u drugi da bi se mogao robot pomaknuti, probati implementirati s move_base i move_base_simple
    #action server result
    def __init__(self):
    smach.State.__init__(self,
                        outcomes=['succeeded','aborted','preempted'],
                        output_keys=['action_out')

    self.pose_msgs = PoseStamped()

    self.move_base_goal = MoveBaseGoal()

    def dataPose(data):
    try:
        self.pose_msgs = data
        self.pose_msgs.pose.position.z = 0.0

        self.move_base_goal.target_pose.header.frame_id = data.header.frame_id
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_goal.target_pose.pose.position.x = data.pose.position.x
        self.move_base_goal.target_pose.pose.position.y = data.pose.position.y
        self.move_base_goal.target_pose.pose.orientation.z = data.pose.orientation.z
        self.move_base_goal.target_pose.pose.orientation.w = data.pose.orientation.w # 1.0

        rospy.loginfo(" msgs %s ",self.pose_msgs)
        rospy.loginfo(" action %s ",self.pose_action)
    except:
        rospy.loginfo("error happens in data Pose")
    
    def execute(self):
    rospy.loginfo('Geting human data from topic /detected_human_pose')
    sub = rospy.Subscriber('/detected_human_pose', PoseStamped, dataPose) 
    rate = rospy.Rate(10) # 10hz
    userdata.action_out = self.move_base_goal
    return self.move_base_goal
