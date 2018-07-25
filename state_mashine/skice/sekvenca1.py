#!/usr/bin/env python

import rospy
import smach
#import smach_ros
from std_srvs.srv import Empty
from smach_ros import *
from std_msgs.msg import String
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

#roslaunch pepper_interaction pepper_robot.launch   
#rosrun map_server map_saver -f mapa
#rosrun map_server map_saver mapa.yaml
#roslaunch pepper_interaction pl... tab
#rosrun pepper_interaction basic_awareness_server.py
#rosrun pepper_interaction marius1.py
# treba napraviti mode koji cita s topica /human detected. topic je definiran unutar basic awerenes servera.


def main():

    rospy.init_node('sekvenca1')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('Wake_up', ServiceState('/pepper_robot/pose/wakeup', Empty),
                                transitions={'succeeded':'',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

       





        sq = smach.Sequence(
            outcomes = ['succeeded','aborted','preempted'],
            connector_outcome = 'succeeded')
            with sq:
                smach.Sequence.add('MOVE_ARM_GRAB_PRE', MoveVerticalGripperPoseActionState())
                smach.Sequence.add('MOVE_GRIPPER_OPEN', MoveGripperState(GRIPPER_MAX_WIDTH))
                smach.Sequence.add('MOVE_ARM_GRAB',     MoveVerticalGripperPoseActionState())
                smach.Sequence.add('MOVE_GRIPPER_CLOSE', MoveGripperState(grab_width))
                smach.Sequence.add('MOVE_ARM_GRAB_POST', MoveVerticalGripperPoseActionState())






# sekvenca1:
#           tts
#           pokret koji je sprzen u chorographu 
            #pokret koji je sekvenca za gripper
            # kombinacija sinkronizacija tts+ pokret     #govor preko servisa pozvati,                  












        # Go to rest pose
        smach.StateMachine.add('Rest', ServiceState('/pepper_robot/pose/rest', Empty),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
    ###########################################
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
