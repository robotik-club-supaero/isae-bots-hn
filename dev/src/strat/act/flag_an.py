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

class deplacement(smach.State):
    
    def __init__(self):
        smach.State.__init__(  self,
                            outcomes=['fail','success','preempted'],
                            input_keys=[],
                            output_keys=[])
    
    def execute(self, userdata):
        
        print("Entering deplacement")
        time.sleep(2)
        print("Exiting deplacement")
        
        return 'success'
        

class OpenDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
                
        
    def execute(self, userdata):
        
        print("Entering OpenGrab")
        time.sleep(2)
        print("Exiting OpenGrab")
        
        return 'success'
    
    
class CloseDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        
        print("Entering CloseGrab")
        time.sleep(2)
        print("Exiting CloseGrab")
                
        return 'success'

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

    pickupplant = smach.StateMachine(outcomes=['fail','success','preempted'])
    with sm:
        smach.StateMachine.add('PICKUPPLANT', pickupplant,
                                transitions={'success':'exit all','fail':'exit all','preempted':'exit preempted'})
  
  
    pick_up_plant_sequence = smach.Sequence(  # sequence container
        input_keys = [],
        output_keys = [],
        outcomes = ['success', 'fail', 'preempted'],
        connector_outcome = 'success')
    
    grab_width = 2
    
    with pick_up_plant_sequence:  # add states to the sequence #TODO define elsewhere
        smach.Sequence.add('OPEN_DOORS', OpenDoors())
        smach.Sequence.add('CLOSE_DOORS', CloseDoors())

    pick_up_plant_concurrence = smach.Concurrence(outcomes=['success', 'fail', 'preempted'],
                                default_outcome='fail',
                                outcome_map={'success': { 'PICKUP_PLANT_SEQ':'success','DEPLACEMENT':'success'},
                                             'preempted' : {'PICK_UP_PLANT_SEQ':'preempted'},
                                             'preempted' : {'DEPLACEMENT' : 'preempted'}})
    
    # Open the container
    with pick_up_plant_concurrence:
        # Add states to the container
        smach.Concurrence.add('PICKUP_PLANT_SEQ', pick_up_plant_sequence)
        smach.Concurrence.add('DEPLACEMENT', deplacement())
    
    
    with pickupplant:
        smach.StateMachine.add('DEPLACEMENT_2', deplacement(),
                               transitions = {'success':'PICKUP_PLANTS_CONC','fail':'fail','preempted':'preempted'})
        smach.StateMachine.add('PICKUP_PLANTS_CONC', pick_up_plant_concurrence, 
                               transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
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
