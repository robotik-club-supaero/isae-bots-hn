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

import os, sys, inspect
import time
import smach

from an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, WAIT_TIME, R_APPROACH_PLANTS
from an_comm import callback_action_pub, add_score, doors_pub, elevator_pub, remove_obs, get_pickup_id, HardwareOrder, LoadDetectorCallback
from an_utils import log_info, log_warn, log_errs, log_fatal, debug_print, debug_print_move

from an_sm_states.sm_displacement import Displacement, Approach, colored_approach
from an_sm_states.sm_waiting import ObsWaitingOnce

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import PLANTS_POS, ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class OpenDoors(HardwareOrder):
    
    def __init__(self):
        super().__init__(doors_pub, 'cb_doors', DoorOrder.OPEN, DoorCallback.PENDING, DoorCallback.OPEN)
    
    def execute(self, userdata):
        debug_print('c', "Request to open doors")
        return super().execute(userdata)
    
class CloseDoors(HardwareOrder):
    
    def __init__(self):
        super().__init__(doors_pub, 'cb_doors', DoorOrder.CLOSE, DoorCallback.PENDING, DoorCallback.CLOSED)

    def execute(self, userdata):
        debug_print('c', "Request to close doors")
        return super().execute(userdata)

class CalcPositionningPlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):  
        plants_id = get_pickup_id("plants", userdata)
        (xp, yp) = PLANTS_POS[plants_id]

        userdata.next_move = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.INITIAL)
                
        return 'success'
    
class CalcTakePlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):              
        plants_id = get_pickup_id("plants", userdata)
        remove_obs.publish(f"plant{plants_id}") # FIXME if action fails, obstacle is not restored

        (xp, yp) = PLANTS_POS[plants_id]

        userdata.next_move = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.FINAL)

        return 'success'
    
class RisePlants(HardwareOrder):
    
    def __init__(self):
        super().__init__(elevator_pub, 'cb_elevator', ElevatorOrder.MOVE_UP, ElevatorCallback.PENDING, ElevatorCallback.UP)
        
    def execute(self, userdata):        
        debug_print('c', "Request to move elevator up")
        return super().execute(userdata)
       
class PickupPlantsEnd(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries plants
        callback_action_pub.publish(exit=ActionResult.SUCCESS, reason='success')
        
        return 'success'
        

#################################################################
#                                                               #
#                        SM STATE : PICKUP_PLANTS               #
#                                                               #
#################################################################

pickupPlant = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors','next_move','robot_pos','cb_depl','next_action','cb_elevator','color'],
                                     output_keys=['cb_doors','next_move','cb_depl','cb_elevator'])
    
    
    
pickUpPlantSequence = smach.Sequence(  # sequence container
    input_keys = ['cb_doors','cb_elevator'],
    output_keys = ['cb_doors','cb_elevator'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')

deplSequence = smach.Sequence(  # sequence container
    input_keys = ['next_move','cb_depl','robot_pos','next_action','color'],
    output_keys = ['cb_depl','next_move'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')


with pickUpPlantSequence:  # add states to the sequence #TODO define elsewhere
    smach.Sequence.add('OPEN_DOORS', OpenDoors())
    smach.Sequence.add('KEEP_OPEN', ObsWaitingOnce(wait_time=2))
    smach.Sequence.add('CLOSE_DOORS', CloseDoors()) # TODO check the robot has actually picked up plants
    smach.Sequence.add('RISE_PLANTS',RisePlants())
    
with deplSequence:
    smach.Sequence.add('CALC_TAKE_PLANT', CalcTakePlants())
    smach.Sequence.add('DEPL_TAKE_PLANTS', Displacement())
    
    
pickUpPlantConcurrence = smach.Concurrence(outcomes=['success', 'fail', 'preempted'],
                            input_keys=['cb_doors','next_move','robot_pos','cb_depl','cb_elevator','next_action','color'],
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
                            transitions = {'success':'PICKUP_PLANT_END', 'fail':'fail', 'preempted':'preempted'}
                            )
    smach.StateMachine.add('PICKUP_PLANT_END', PickupPlantsEnd(), 
                            transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
                            )