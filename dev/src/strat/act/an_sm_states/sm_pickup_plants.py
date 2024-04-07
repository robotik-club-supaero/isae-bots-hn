# -*- coding: utf-8 -*-
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 

# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import sys
import time
import smach
import math
from an_const import *
from an_comm import callback_action_pub, add_score, doors_pub, elevator_pub
from an_utils import log_info, log_warn, log_errs, log_fatal
from an_sm_states.sm_displacement import Displacement, set_next_destination
from geometry_msgs.msg import Quaternion, Pose2D

from an_sm_states.sm_displacement import Displacement, set_next_destination

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

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
        
        time.sleep(r/v)
        
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
                                input_keys=['robot_pos'],
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
    
class RisePlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_elevator'],
                                output_keys=['cb_elevator'])
        
    def execute(self, userdata):
        
        userdata.cb_elevator[0] = ElevatorCallback.PENDING
        
        elevator_pub.publish(ElevatorCallback.UP)
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_doors[0] == ElevatorCallback.UP:
                return 'success'
            elif userdata.cb_doors[0] == ElevatorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_warn("Timeout")
        
        return 'fail'
        

#################################################################
#                                                               #
#                        SM STATE : PICKUP_PLANTS               #
#                                                               #
#################################################################

pickupPlant = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors','next_move','robot_pos','cb_depl','next_pos'],
                                     output_keys=['cb_doors','next_move','cb_depl'])
    
    
    
pickUpPlantSequence = smach.Sequence(  # sequence container
    input_keys = ['cb_doors','cb_elevator'],
    output_keys = ['cb_doors','cb_elevator'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')

deplSequence = smach.Sequence(  # sequence container
    input_keys = ['next_move','cb_depl','robot_pos'],
    output_keys = ['cb_depl','next_move'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')


with pickUpPlantSequence:  # add states to the sequence #TODO define elsewhere
    smach.Sequence.add('OPEN_DOORS', OpenDoors())
    smach.Sequence.add('CLOSE_DOORS', CloseDoors())
    smach.Sequence.add('RISE_PLANTS',RisePlants())
    
with deplSequence:
    smach.Sequence.add('CALC_TAKE_PLANT', CalcTakePlants())
    smach.Sequence.add('DEPL_TAKE_PLANTS', Displacement())
    
    
pickUpPlantConcurrence = smach.Concurrence(outcomes=['success', 'fail', 'preempted'],
                            input_keys=['cb_doors','next_move','robot_pos','cb_depl','cb_elevator'],
                            output_keys=['cb_doors','next_move','cb_depl','cb_elevator'],
                            default_outcome='fail',
                            outcome_map={'success': { 'PICKUP_PLANT_SEQ':'success','DEPL_SEQ':'success'},
                                            'preempted' : {'PICK_UP_PLANT_SEQ':'preempted'},
                                            'preempted' : {'DEPL_SEQ' : 'preempted'}})

with pickUpPlantConcurrence:
    # Add states to the container
    smach.Concurrence.add('PICKUP_PLANT_SEQ', pickUpPlantSequence)
    smach.Concurrence.add('DEPL_SEQ', deplSequence)
    
with pickupPlant:
    smach.StateMachine.add('CALC_POSITIONING_PLANTS', CalcPositionningPlants(),
                            transitions = {'success':'DEPL_POSITIONING_PLANTS','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('DEPL_POSITIONING_PLANTS', Displacement(),
                            transitions = {'success':'PICKUP_PLANTS_CONC','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('PICKUP_PLANTS_CONC', pickUpPlantConcurrence, 
                            transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
                            )