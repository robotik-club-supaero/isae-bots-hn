#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

########## IMPORTS ##########
import rospy
import smach_ros
import smach
import time
from Enum import enum

from rospy import logwarn, loginfo, logerr, logfatal
from std_msgs.msg import Int16
from an_utils import log_info, log_warn, log_errs, log_fatal

########## CONSTANTES ##########
WAIT_TIME = 50



class DoorCallback(enum):
    UNKNOWN = -2
    PENDING = -1
    CLOSED = 0
    OPEN = 1
    BLOCKED = 2
    
class DoorOrder(enum):
    OPEN = 0
    CLOSE = 1

class DspCallback(enum):
    UNKNOWN = -2
    PENDING = -1
    ARRIVED = 0
    OBSTACLE = 1
    OBSTACLE_ON_TARGET = 2
    ERROR_ASSERV = 3
    

########## FONCTIONS DES SUBSCRIBERS ##########

# def callback_stop(msg):
#     if msg.data == 1:
#         sm.userdata.stop = True

def cb_doors_fct(msg):
    if msg.data == 1:
        sm.userdata.cb_doors[0] = DoorCallback.OPEN
    elif msg.data == 0:
        sm.userdata.cb_doors[0] = DoorCallback.CLOSED
    else:
        sm.userdata.cb_doors[0] = DoorCallback.UNKNOWN
            
        
def callback_dsp_fct(msg):
    if msg.data == DspCallback.ARRIVED:
        sm.callback_dsp[0]
        
        

########## ETATS ##########

class deplacement(smach.State):
    
    def __init__(self):
        smach.State.__init__(  self,
                            outcomes=['fail','success','preempted'],
                            input_keys=[],
                            output_keys=[])
    
    def execute(self, userdata):
        
        return 'fail'
        

class OpenDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_doors'],
                                output_keys=['cb_doors'])
                
        
    def execute(self, userdata):
        
        userdata.cb_doors[0] = DoorCallback.PENDING
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_doors[0] == DoorCallback.OPEN:
                return 'success'
            elif userdata.cb_doors[0] == DoorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_warn("Timeout")
        
        return 'fail'
    
    
class CloseDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_doors'],
                                output_keys=['cb_doors'])
        
    def execute(self, userdata):
        
        userdata.cb_doors[0] = DoorCallback.PENDING

        # publish close doors order
        doors_pub.publish(DoorOrder.CLOSE)
                
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_doors[0] == DoorCallback.CLOSED:
                return 'success'
            elif userdata.cb_doors[0] == DoorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_info("Timeout")
        
        return 'fail'

########## MAIN ##########
def main():

    print("\nAction node launched\n")

    global sm 	# la machine a etat (state machine)
    sm = smach.StateMachine(outcomes=['exit all', 'exit preempted'])  # exit all -> sortie de la mae

    # sm.userdata.stop = False
    sm.userdata.cb_doors = [False]

    #################
    # Publishers & Subscribers
    global flag_pub
    flag_pub = rospy.Publisher('/flag_alert', Int16, queue_size = 10, latch= True)

    # global flag_sub
    # flag_sub = rospy.Subscriber('/flag_stop', Int16, callback_stop)
    global doors_pub
    doors_pub = rospy.Publisher('/act/order/doors', Int16, queue_size = 10, latch= True)
    doors_sub = rospy.Subscriber('/act/callback/doors', Int16, cb_doors_fct)
        
    depl_sub = rospy.Subscriber('/dsp/callback', Int16, callback_dsp)

    # Remplissage de la MAE

    pickupplant = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors'],
                                     output_keys=['cb_doors'])
    with sm:
        smach.StateMachine.add('PICKUPPLANT', pickupplant,
                                transitions={'success':'exit all','fail':'exit all','preempted':'exit preempted'})
  
  
    pick_up_plant_sequence = smach.Sequence(  # sequence container
        input_keys = ['cb_doors'],
        output_keys = ['cb_doors'],
        outcomes = ['success', 'fail', 'preempted'],
        connector_outcome = 'success')
    
    grab_width = 2
    
    with pick_up_plant_sequence:  # add states to the sequence #TODO define elsewhere
        smach.Sequence.add('OPEN_DOORS', OpenDoors())
        smach.Sequence.add('CLOSE_DOORS', CloseDoors())

    pick_up_plant_concurrence = smach.Concurrence(outcomes=['success', 'fail', 'preempted'],
                                input_keys=['cb_doors'],
                                output_keys=['cb_doors'],
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
