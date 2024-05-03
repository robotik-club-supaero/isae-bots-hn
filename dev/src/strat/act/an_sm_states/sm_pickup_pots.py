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

import smach
import math
import time

from .sm_pickup_plants import CloseDoors, OpenDoors
from an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, WAIT_TIME, R_APPROACH_POTS, R_TAKE_POTS, DspOrderMode
from an_utils import debug_print
from strat_const import POTS_POS, ActionResult
from an_sm_states.sm_displacement import Displacement, Approach, colored_approach_with_angle
from an_sm_states.sm_waiting import ObsWaitingOnce
from an_comm import get_pickup_id, elevator_pub, callback_action_pub, remove_obs, HardwareOrder

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPots(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):    
        pots_id = get_pickup_id("pots", userdata)
        remove_obs.publish(f"pot{pots_id}") # FIXME if action fails, obstacle is not restored

        xp, yp, thetap = POTS_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_APPROACH_POTS)
             
        return 'success'


class CalcTakePots(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):    
        pots_id = get_pickup_id("pots", userdata)

        xp, yp, thetap = POTS_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_TAKE_POTS)
             
        return 'success'

class PotPlants(HardwareOrder):
    
    def __init__(self):
        super().__init__(elevator_pub, 'cb_elevator', ElevatorOrder.MOVE_DOWN, ElevatorCallback.PENDING, ElevatorCallback.DOWN)

    def execute(self, userdata):        
        debug_print('c', "Request to move elevator down")
        return super().execute(userdata)

    
 
class PickupPotsEnd(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries pots
        callback_action_pub.publish(exit=ActionResult.SUCCESS, reason='success')        
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : PICKUP_POTS                 #
#                                                               #
#################################################################

pickupPot = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors','next_move','robot_pos','cb_depl','next_action','cb_elevator','color'],
                                     output_keys=['cb_doors','next_move','cb_depl','cb_elevator'])
    
pickUpPotSequence = smach.Sequence(
    input_keys = ['cb_doors','cb_elevator'],
    output_keys = ['cb_doors','cb_elevator'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')

deplSequence = smach.Sequence(
    input_keys = ['next_move','cb_depl','robot_pos','next_action','color'],
    output_keys = ['cb_depl','next_move'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')

with pickUpPotSequence: 
    smach.Sequence.add('OPEN_DOORS', OpenDoors())
    smach.Sequence.add('KEEP_OPEN', ObsWaitingOnce(wait_time=0.2))
    smach.Sequence.add('CLOSE_DOORS', CloseDoors()) # gather pots # TODO: add delay before closing ?
    # TODO check the robot has actually picked up pots
    smach.Sequence.add('POT_PLANTS', PotPlants()) # put grabbed plants into pots

with deplSequence:
    smach.Sequence.add('CALC_TAKE_POTS', CalcTakePots())
    smach.Sequence.add('DEPL_TAKE_POTS', Displacement())
    
pickUpPlantConcurrence = smach.Concurrence(outcomes=['success', 'fail', 'preempted'],
                            input_keys=['cb_doors','next_move','robot_pos','cb_depl','cb_elevator','next_action','color'],
                            output_keys=['cb_doors','next_move','cb_depl','cb_elevator'],
                            default_outcome='fail',
                            outcome_map={'success': { 'PICKUP_POT_SEQ':'success','DEPL_SEQ':'success'},
                                            'preempted' : {'PICKUP_POT_SEQ':'preempted'},
                                            'preempted' : {'DEPL_SEQ' : 'preempted'}})

with pickUpPlantConcurrence:
    smach.Concurrence.add('PICKUP_POT_SEQ', pickUpPotSequence)
    smach.Concurrence.add('DEPL_SEQ', deplSequence)
    

with pickupPot:
    smach.StateMachine.add('CALC_POSITIONING_POTS', CalcPositionningPots(),
                            transitions = {'success':'DEPL_POSITIONING_POTS','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('DEPL_POSITIONING_POTS', Displacement(),
                            transitions = {'success':'PICKUP_POTS_CONC','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('PICKUP_POTS_CONC', pickUpPlantConcurrence, 
                            transitions = {'success':'PICKUP_POTS_END', 'fail':'fail', 'preempted':'preempted'}
                            )
    smach.StateMachine.add('PICKUP_POTS_END', PickupPotsEnd(), 
                            transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
                            )