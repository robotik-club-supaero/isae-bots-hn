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
from std_msgs.msg      import Empty

from .sm_pickup_plants import CloseDoors, OpenDoors
from an_const import DoorCallback, DoorOrder, WAIT_TIME, R_APPROACH_POTS, DspOrderMode, DspCallback
from an_utils import debug_print, log_errs, log_warn, log_info
from strat_const import DEPOSIT_POS
from strat_utils import adapt_pos_to_side
from an_sm_states.sm_displacement import Displacement, Approach, colored_approach_with_angle, DISP_TIMEOUT
from an_comm import get_pickup_id, callback_action_pub, deposit_pub, disp_pub

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
        pots_id = get_pickup_id("deposit pots", userdata)

        xp, yp, thetap = DEPOSIT_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_APPROACH_POTS)
             
        return 'success'

class MoveBackwardsStraight(smach.State):
    
    def __init__(self, dist):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_depl', 'next_action', 'color'],
                                output_keys=['cb_depl'])
        self._dist = dist
        
    def execute(self, userdata):
        
        debug_print('c', "Request to move backwards")
        
        userdata.cb_depl[0] = DspCallback.PENDING

        pots_id = get_pickup_id("deposit pots", userdata)
        x, y, theta = adapt_pos_to_side(*DEPOSIT_POS[pots_id], userdata.color)
    
        xd = x - self._dist * math.cos(theta)
        yd = y - self._dist * math.sin(theta)

        deposit_pub.publish(Empty())
        disp_pub.publish(Quaternion(xd, yd, theta, DspOrderMode.BACKWARDS))
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < DISP_TIMEOUT:           
            time.sleep(0.01)

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if userdata.cb_depl[0] == DspCallback.ERROR_ASSERV:
                log_errs("Displacement result: error asserv.")
                return 'fail'

            if userdata.cb_depl[0] == DspCallback.SUCCESS:
                log_info('Displacement result: success displacement')
                return 'success'

        # timeout
        log_warn("Timeout")
        
        return 'fail'
    
 
class DepositPotsEnd(smach.State):
    
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
#                        SM STATE : DEPOSIT_POTS                 #
#                                                               #
#################################################################

depositPot = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors','next_move','robot_pos','cb_depl','next_action','color'],
                                     output_keys=['cb_doors','next_move','cb_depl'])
    
depositPotSequence = smach.Sequence(
    input_keys = ['cb_doors','next_move','robot_pos','cb_depl','next_action','color'],
    output_keys = ['cb_doors','next_move','cb_depl'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')


with depositPotSequence: 
    smach.Sequence.add('CALC_POSITIONING_POTS', CalcPositionningPots())
    smach.Sequence.add('DEPL_POSITIONING_POTS', Displacement())
    smach.Sequence.add('OPEN_DOORS', OpenDoors())
    smach.Sequence.add('RELEASE_POTS', MoveBackwardsStraight(dist=R_APPROACH_POTS+50)) # TODO change dist
    smach.Sequence.add('CLOSE_DOORS', CloseDoors())
    smach.Sequence.add('DEPOSIT_POTS_END',  DepositPotsEnd())

with depositPot:
    smach.StateMachine.add('INNER_SEQ', depositPotSequence,
                            transitions = {'success':'success','fail':'fail','preempted':'preempted'})