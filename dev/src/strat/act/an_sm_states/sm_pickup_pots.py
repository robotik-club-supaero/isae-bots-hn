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
from an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, WAIT_TIME, R_APPROACH_POTS, DspOrderMode
from an_utils import debug_print
from strat_const import POTS_POS
from an_sm_states.sm_displacement import Displacement, Approach, colored_approach_with_angle
from an_comm import get_pickup_id, elevator_pub, callback_action_pub, HardwareOrder

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

        xp, yp, thetap = POTS_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_APPROACH_POTS)
             
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
        callback_action_pub.publish(exit=1, reason='success')
        
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


with pickUpPotSequence: 
    smach.Sequence.add('OPEN_DOORS', OpenDoors())
    smach.Sequence.add('CLOSE_DOORS', CloseDoors()) # gather pots # TODO: add delay before closing ?
    smach.Sequence.add('POT_PLANTS', PotPlants()) # put grabbed plants into pots

with pickupPot:
    smach.StateMachine.add('CALC_POSITIONING_POTS', CalcPositionningPots(),
                            transitions = {'success':'DEPL_POSITIONING_POTS','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('DEPL_POSITIONING_POTS', Displacement(),
                            transitions = {'success':'PICKUP_POTS_SEQ','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('PICKUP_POTS_SEQ', pickUpPotSequence, 
                            transitions = {'success':'PICKUP_POTS_END', 'fail':'fail', 'preempted':'preempted'}
                            )
    smach.StateMachine.add('PICKUP_POTS_END', PickupPotsEnd(), 
                            transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
                            )