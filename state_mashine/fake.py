#!/usr/bin/env python
#!/usr/bin/env python
# coding: utf8
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from smach_ros import *
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PickupActionGoal
import time


class Fake(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['Izvrseno'],
                                    input_keys=['time'])
       
    def execute(self, userdata):
        print userdata.time
        time.sleep(userdata.time) 
        return 'succeeded'


class Fake_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['Izvrseno','vocabulary_1','vocabulary_2','vocabulary_3','vocabulary_4','vocabulary_5','vocabulary_6','odmori'],
                                    input_keys=['time'])
                              

    def execute(self, userdata):
        print userdata.time
        time.sleep(userdata.time) 
        return 'succeeded'

def main():

    rospy.init_node('Stateovi')

    # Create a SMACH state machine
    sm = smach.StateMachine(['Izvrseno'])

    sm.userdata.time = 3.0
    sm.userdata.body_pose = "Stand"
    sm.userdata.vocabulary_data = 'yes, please, no, love, kill'
    sm.userdata.behavior = 'animations/Stand/Gestures/Stretch_2'
    sm.userdata.joint_names = ''
    sm.userdata.joint_points = [0]
    sm.userdata.word = ""
    sm.userdata.word_conf = 0

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('Probudi robota', Fake(),
                                transitions={'Izvrseno':'Izgovori uvodnu recenicu uz pokrete_0'})
     


        # Wake up robot 
        smach.StateMachine.add('Izgovori recenicu uz pokrete_0', Fake(),
                                transitions={'Izvrseno':'Postavi zainteresirani mod rada'})
     

        # Wake up robot 
        smach.StateMachine.add('Postavi zainteresirani mod rada', Fake(),
                                transitions={'Izvrseno':'Postavi pracenje osobe glavom'})
     

        # Wake up robot 
        smach.StateMachine.add('Postavi pracenje osobe glavom', Fake(),
                                transitions={'Izvrseno':'Zapocni pratiti osobu'})
     

        # Wake up robot 
        smach.StateMachine.add('Zapocni pratiti osobu', Fake(),
                                transitions={'Izvrseno':'Ocitaj poziciju osobe'})
     

        # Wake up robot 
        smach.StateMachine.add('Ocitaj poziciju osobe', Fake(),
                                transitions={'Izvrseno':'Prestani pratiti osobu'})
     

        # Wake up robot 
        smach.StateMachine.add('Prestani pratiti osobu', Fake(),
                                transitions={'Izvrseno':'Izgovori recenicu uz pokrete_1'})
     

        # Wake up robot 
        smach.StateMachine.add('Izgovori recenicu uz pokrete_1', Fake(),
                                transitions={'Izvrseno':'Pricekaj'})
     

        # Wake up robot 
        smach.StateMachine.add('Pricekaj', Fake(),
                                transitions={'Izvrseno':'Navigiraj do osobe'})

        # Wake up robot 
        smach.StateMachine.add('Navigiraj do osobe', Fake(),
                                transitions={'Izvrseno':'Postavi pracenje povremenim pokretima'})
     

        # Wake up robot 
        smach.StateMachine.add('Postavi pracenje povremenim pokretima', Fake(),
                                transitions={'Izvrseno':'Izgovori uvodnu recenicu uz pokrete_2'})
     

        # Wake up robot 
        smach.StateMachine.add('Izgovori uvodnu recenicu uz pokrete_2', Fake(),
                                transitions={'Izvrseno':'Stavi rijeci u vokabular'})
     

        # Wake up robot 
        smach.StateMachine.add('Stavi rijeci u vokabular', Fake(),
                                transitions={'Izvrseno':'Pokreni ASR'})
     

        # Wake up robot 
        smach.StateMachine.add('Pokreni ASR', Fake(),
                             transitions={'Izvrseno':'Prepoznaj rijec'})
        # Wake up robot 
        smach.StateMachine.add('Prepoznaj rijec', Fake_2(),
                          transitions={'Izvrseno':'Prepoznaj rijec',
                                             'vocabulary_1':'Radost',
                                             'vocabulary_2':'Ljutnja',
                                             'vocabulary_3':'Strah',
                                             'vocabulary_4':'Tuga',
                                             'vocabulary_5':'Gadjenje',
                                             'vocabulary_6':'Iznenadjenje',
                                             'odmori':'odmori'})
                                

        # Wake up robot 
        smach.StateMachine.add('vocabulary_1', Fake(),
                                transitions={'Izvrseno':'Prepoznaj rijec'})
     

        # Wake up robot 
        smach.StateMachine.add('vocabulary_2', Fake(),
                                transitions={'Izvrseno':'Prepoznaj rijec'})
     

        # Wake up robot 
        smach.StateMachine.add('vocabulary_3', Fake(),
                                transitions={'Izvrseno':'Prepoznaj rijec'})
     

        # Wake up robot 
        smach.StateMachine.add('vocabulary_4', Fake(),
                                transitions={'Izvrseno':'Prepoznaj rijec'})
     

        # Wake up robot 
        smach.StateMachine.add('vocabulary_5', Fake(),
                                transitions={'Izvrseno':'Prepoznaj rijec'})
     

        # Wake up robot 
        smach.StateMachine.add('vocabulary_6', Fake(),
                                transitions={'Izvrseno':'Prepoznaj rijec'})
     


        # Go to rest pose
        smach.StateMachine.add('odmori', Fake(),
                                transitions={'Izvrseno':'Izvrseno'})

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
