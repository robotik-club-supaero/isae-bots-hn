#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

########## IMPORTS ##########
import rospy
import smach_ros
import smach
import time

from rospy import logwarn, loginfo, logerr, logfatal
from std_msgs.msg import Int16

########## CONSTANTES ##########
WAIT_TIME = 5



########## FONCTIONS DES SUBSCRIBERS ##########

def callback_stop(msg):
    if msg.data == 1:
        sm.userdata.stop = True

########## ETATS ##########
class flag(smach.State):
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['done', 'stop'],
                                input_keys=['stop'],
                                output_keys=['stop'])
        
    def execute(self, userdata):

        #################  # Action : attendre et lever le flag, sortir si besoin
        begin = time.time()
        while time.time()-begin < WAIT_TIME:

            if userdata.stop:
                return 'stop'

            time.sleep(0.01)
        
        #Lever le bras
        flag_pub.publish(data=1)

        #################
        return 'done'	# sortie de la sm





class OpenGrab(smach.State):
    
    def __init__(self, grab_width):
        smach.State.__init__(	self,
                                outcomes=['success', 'abort', 'preempted'],
                                input_keys=['stop', grab_width],
                                output_keys=['stop'])
        self.grab_width = grab_width
                
        
    def execute(self, userdata):
        
        print("Entering OpenGrab with grab width ", self.grab_width)
        time.sleep(2)
        print("Exiting OpenGrab")
        
        return
    
    
class CloseGrab(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['success', 'abort', 'preempted'],
                                input_keys=['stop'],
                                output_keys=['stop'])
        
    def execute(self, userdata):
        
        print("Entering CloseGrab")
        time.sleep(2)
        print("Exiting CloseGrab")
                
        return
    
    
    
    
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        
        time.sleep(8)
        
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        
        time.sleep(10)
        
        return 'outcome1'


########## MAIN ##########
def main():

    print("\nAction node launched\n")

    global sm 	# la machine a etat (state machine)
    sm = smach.StateMachine(outcomes=['exit all', 'exit preempted'])  # exit all -> sortie de la mae

    sm.userdata.stop = False

    #################
    # Publishers & Subscribers
    global flag_pub
    flag_pub = rospy.Publisher('/flag_alert', Int16, queue_size = 10, latch= True)

    global flag_sub
    flag_sub = rospy.Subscriber('/flag_stop', Int16, callback_stop)

    # Remplissage de la MAE

    with sm:
        smach.StateMachine.add('FLAG', flag(),
                                transitions={'done':'FLAG','stop':'exit all'})
  
  
    grab_sequence = smach.Sequence(  # sequence container
        input_keys = [],
        output_keys = [],
        outcomes = ['success', 'abort', 'preempted'],
        connector_outcome = 'success')
    
    grab_width = 2
    
    with grab_sequence:  # add states to the sequence #TODO define elsewhere
        smach.Sequence.add('OPEN_GRAB', OpenGrab(grab_width))
        smach.Sequence.add('CLOSE_GRAB', CloseGrab())


    # add the sequence container to the sm
    with sm:
        smach.StateMachine.add('GRAB_SEQ', grab_sequence, 
                               transitions = {'success':'GRAB_SEQ', 'abort':'exit all', 'preempted':'exit preempted'}
                               )
        
        
    grab_concurrence = smach.Concurrence(outcomes=['success', 'abort', 'preempted'],
                                default_outcome='abort',
                                outcome_map={'success': { 'FOO':'outcome2','BAR':'outcome1'},
                                             'abort': { 'FOO':'outcome2'},
                                             'preempted' : {'BAR':'outcome1'}})
    
    # Open the container
    with grab_concurrence:
        # Add states to the container
        smach.Concurrence.add('FOO', Foo())
        smach.Concurrence.add('BAR', Bar())
    
    
    with sm:
        smach.StateMachine.add('GRAB_CONC', grab_concurrence, 
                               transitions = {'success':'GRAB_SEQ', 'abort':'exit all', 'preempted':'exit preempted'}
                               )

    #################




########## APPEL DE LA MAE ##########
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('flag_an', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print('\nSortie de la machine a etats\n')
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('action_node')
    main()
