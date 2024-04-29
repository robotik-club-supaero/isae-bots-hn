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
from geometry_msgs.msg import Quaternion

from .sm_pickup_plants import CloseDoors, OpenDoors
from an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, WAIT_TIME, R_APPROACH_POTS, DspOrderMode
from an_utils import adapt_pos_to_side, debug_print
from strat_const import POTS_POS
from an_sm_states.sm_displacement import Displacement, Approach, colored_approach
from an_comm import get_pickup_id, elevator_pub, callback_action_pub

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPots(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):    
        x, y = userdata.robot_pos.x, userdata.robot_pos.y
        pots_id = get_pickup_id("pots", userdata)
        (xp, yp, thetap) = POTS_POS[pots_id]
        
        x_d, y_d, t_d = adapt_pos_to_side(xp, yp, thetap, userdata.color)
        userdata.next_move = Quaternion(x_d - R_APPROACH_POTS * math.cos(thetap), y_d - R_APPROACH_POTS * math.sin(thetap), thetap, DspOrderMode.AVOIDANCE.value)
                
        return 'success'

class PotPlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_elevator'],
                                output_keys=['cb_elevator'])
        
    def execute(self, userdata):
        
        debug_print('c', "Request to move elevator down")
        
        userdata.cb_elevator[0] = ElevatorCallback.PENDING
        
        elevator_pub.publish(ElevatorOrder.MOVE_DOWN.value)
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_elevator[0] == ElevatorCallback.DOWN:
                return 'success'
            elif userdata.cb_elevator[0] == ElevatorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_warn("Timeout")
        
        return 'fail'
    
 
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
    smach.Sequence.add('CLOSE_DOORS', CloseDoors()) # gather pots
    smach.Sequence.add('POT_PLANTS', PotPlants()) # put grabbed plants into pots

with pickupPot:
    smach.StateMachine.add('CALC_POSITIONING_POTS', CalcPositionningPots(),
                            transitions = {'success':'DEPL_POSITIONING_POTS','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('DEPL_POSITIONING_POTS', Displacement(),
                            transitions = {'success':'PICKUP_POTS_SEQ','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('PICKUP_POTS_SEQ', pickUpPotSequence, 
                            transitions = {'success':'PICKUP_POTS_END', 'fail':'PICKUP_POTS_END', 'preempted':'preempted'}
                            )
    smach.StateMachine.add('PICKUP_POTS_END', PickupPotsEnd(), 
                            transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
                            )