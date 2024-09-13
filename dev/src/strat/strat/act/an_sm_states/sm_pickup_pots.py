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

from ..an_const import R_APPROACH_POTS, R_TAKE_POTS, DspOrderMode
from ..an_utils import AutoSequence, AutoConcurrence, OpenDoors, CloseDoors, OpenClamp, RiseElevator, DescendElevator
from strat.strat_const import POTS_POS, ActionResult
from .sm_displacement import MoveTo, Approach, colored_approach_with_angle
from .sm_waiting import ObsWaitingOnce

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPots(smach.State):
    
    def __init__(self, node):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['color','next_action'],
                                output_keys=['next_move'])
        self._node = node
        
    def execute(self, userdata):    
        pots_id = self._node.get_pickup_id("pots", userdata)
        self._node.remove_obs.publish(f"pot{pots_id}") # FIXME if action fails, obstacle is not restored
        
        xp, yp, thetap = POTS_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_APPROACH_POTS)
             
        return 'success'


class CalcTakePots(smach.State):
    
    def __init__(self, node):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['color','next_action'],
                                output_keys=['next_move'])
        self._node = node
        
    def execute(self, userdata):    
        pots_id = self._node.get_pickup_id("pots", userdata)

        xp, yp, thetap = POTS_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_TAKE_POTS)
             
        return 'success'
 
class PickupPotsEnd(smach.State):
    
    def __init__(self, callback_action_pub):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries pots
        self._callback_action_pub.publish(exit=ActionResult.SUCCESS, reason='success')        
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : PICKUP_POTS                 #
#                                                               #
#################################################################

class _PickupPlotSequence(AutoSequence):
    def __init__(self, node):
        super().__init__(
            ('OPEN_DOORS', OpenDoors(node)),
            ('KEEP_OPEN', ObsWaitingOnce(wait_time=0.2)),
            ('CLOSE_DOORS', CloseDoors(node)), # gather pots
            # TODO check the robot has actually picked up pots
            ('POT_PLANTS', DescendElevator(node)), # put grabbed plants into pots
            ('RELEASE_PLANTS', OpenClamp(node)),
            ('RISE_ELEVATOR', RiseElevator(node)),
        )

class PickupPlot(AutoSequence):
    def __init__(self, node):
        super().__init__(
            ('DEPL_POSITIONING_POTS', MoveTo(node, CalcPositionningPots(node))),
            ('PICKUP_POTS_CONC', AutoConcurrence(
                DEPL_SEQ = MoveTo(node, CalcTakePots(node)),
                PICKUP_POT_SEQ = _PickupPlotSequence(node),
            )),
            ('PICKUP_POTS_END', PickupPotsEnd(node.callback_action_pub)),
    )