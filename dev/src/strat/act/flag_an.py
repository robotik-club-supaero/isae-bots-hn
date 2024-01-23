#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

########## IMPORTS ##########
import rospy
import smach_ros
import smach
import time
import math
from enum import Enum

from an_utils import log_info
from std_msgs.msg import Int16
from an_utils import log_info, log_warn, log_errs, log_fatal
from geometry_msgs.msg import Quaternion, Pose2D


########## CONSTANTES ##########
WAIT_TIME = 500
order = [0,1]
pos_plant = [(1800,700),(500,2400)]
r = 300

class DoorCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    CLOSED = 0
    OPEN = 1
    BLOCKED = 2
    
class DoorOrder(Enum):
    OPEN = 0
    CLOSE = 1

class DspCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    ARRIVED = 0
    OBSTACLE = 1
    OBSTACLE_ON_TARGET = 2
    ERROR_ASSERV = 3
    
class DspOrder(Enum):
    STOP = 0
    MOVE_STRAIGHT = 1
    

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
    sm.userdata.cb_depl[0] = DspCallback(msg.data)
        
        

########## ETATS ##########

class deplacement(smach.State):
    
    def __init__(self):
        smach.State.__init__(  self,
                            outcomes=['fail','success','preempted'],
                            input_keys=['next_move','cb_depl'],
                            output_keys=['cb_depl'])
    
    def execute(self, userdata):
        
        userdata.cb_depl[0] = DspCallback.PENDING
        
        depl_pub.publish(userdata.next_move)
                
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            # log_info(f"cb_depl : {userdata.cb_depl[0]}")
            
            if userdata.cb_depl[0] == DspCallback.ARRIVED:
                return 'success'
            elif userdata.cb_depl[0] == DspCallback.OBSTACLE:
                depl_pub.publish(DspOrder.STOP)
                return 'fail'
            elif userdata.cb_depl[0] == DspCallback.OBSTACLE_ON_TARGET:
                depl_pub.publish(DspOrder.STOP)
                return 'fail'
            time.sleep(0.1)
            
        # timeout
        log_warn("Timeout")
        
        
        return 'fail'
        

class OpenDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_doors'],
                                output_keys=['cb_doors'])
                
        
    def execute(self, userdata):
        
        userdata.cb_doors[0] = DoorCallback.PENDING
        
        doors_pub.publish(DoorOrder.OPEN)
        
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
    
class CalcPositionningPlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos',],
                                output_keys=['next_move'])
        
    def execute(self, userdata):
        
        delta_y= pos_plant[order[1]][1] - userdata.robot_pos.y
        delta_x= pos_plant[order[1]][0] - userdata.robot_pos.x
        theta= math.atan2(delta_y,delta_x)
        
        
        userdata.next_move = Quaternion(-r*math.cos(theta) + pos_plant[order[1]][0], r*math.sin(theta) + pos_plant[order[1]][1],theta, DspOrder.MOVE_STRAIGHT)
        
        return 'success'
    
class CalcTakePlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):
        
        delta_y= pos_plant[order[1]][1] - userdata.robot_pos.y
        delta_x= pos_plant[order[1]][0] - userdata.robot_pos.x
        theta= math.atan2(delta_y,delta_x)
        userdata.next_move = Quaternion(r*math.cos(theta) + pos_plant[order[1]][0], -r*math.sin(theta) + pos_plant[order[1]][1],theta, DspOrder.MOVE_STRAIGHT)
        
        return 'success'

########## MAIN ##########
def main():

    print("\nAction node launched\n")

    global sm 	# la machine a etat (state machine)
    sm = smach.StateMachine(outcomes=['exit all', 'exit preempted'])  # exit all -> sortie de la mae

    # sm.userdata.stop = False
    sm.userdata.cb_doors = [DoorCallback.UNKNOWN]
    sm.userdata.next_move = Quaternion(0,0,0,0)
    sm.userdata.robot_pos = Pose2D(0,0,0)
    sm.userdata.cb_depl = [DspCallback.UNKNOWN]
    

    #################
    # Publishers & Subscribers
    global flag_pub
    flag_pub = rospy.Publisher('/flag_alert', Int16, queue_size = 10, latch= True)

    # global flag_sub
    # flag_sub = rospy.Subscriber('/flag_stop', Int16, callback_stop)
    global doors_pub
    doors_pub = rospy.Publisher('/act/order/doors', Int16, queue_size = 10, latch= True)
    doors_sub = rospy.Subscriber('/act/callback/doors', Int16, cb_doors_fct)
    
    global depl_pub  
    depl_sub = rospy.Subscriber('/dsp/callback', Int16, callback_dsp_fct)
    depl_pub = rospy.Publisher('/dsp/order/',Quaternion, queue_size = 10, latch= True)

    # Remplissage de la MAE

    pickupplant = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors','next_move','robot_pos','cb_depl'],
                                     output_keys=['cb_doors','next_move','cb_depl'])
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
                                input_keys=['cb_doors','next_move','robot_pos','cb_depl'],
                                output_keys=['cb_doors','next_move','cb_depl'],
                                default_outcome='fail',
                                outcome_map={'success': { 'PICKUP_PLANT_SEQ':'success','DEPL_TAKE_PLANTS':'success'},
                                             'preempted' : {'PICK_UP_PLANT_SEQ':'preempted'},
                                             'preempted' : {'DEPL_TAKE_PLANTS' : 'preempted'}})
    
    # Open the container
    with pick_up_plant_concurrence:
        # Add states to the container
        smach.Concurrence.add('PICKUP_PLANT_SEQ', pick_up_plant_sequence)
        smach.Concurrence.add('DEPL_TAKE_PLANTS', deplacement())
    
    
    with pickupplant:
        smach.StateMachine.add('CALC_POSITIONING_PLANTS', CalcPositionningPlants(),
                               transitions = {'success':'DEPL_POSITIONING_PLANTS','fail':'fail','preempted':'preempted'})
        smach.StateMachine.add('DEPL_POSITIONING_PLANTS', deplacement(),
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
