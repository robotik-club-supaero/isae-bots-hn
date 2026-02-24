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
import yasmin
import math

from config import StratConfig

from ..an_const import *
from ..an_utils import Sequence

from .sm_displacement import MoveTo, approach, Approach
from .sm_waiting import Waiting
from strat.strat_utils import create_end_of_action_msg

from strat.strat_const import ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcTargetPos(yasmin.State):
    """
    SM MOVE : move setup
    """
    def __init__(self):
        super().__init__(outcomes=['preempted','success','fail'])		                   

    def execute(self, userdata):
        if self.is_canceled(): return 'preempted'

        ## Move to parking wait position
        xp, yp, tp = StratConfig(userdata["color"]).wait_park_pos
        reverse = True if userdata["color"] == 1 else False

        # --- If need to go behind, go reverse as defined if angle final is close to initial
        xr, yr, tr = userdata["robot_pos"].x, userdata["robot_pos"].y, userdata["robot_pos"].theta
        opposite = ((xp - xr) * math.cos(tr) + (yp - yr) * math.sin(tr)) < 0
        delta_t = abs((tp % 3.142) - (tr % 3.142))
        if opposite:
            if not reverse:
                if (delta_t < 1.6): reverse = not reverse
        else:
            if reverse:
                if (delta_t < 1.6): reverse = not reverse 
        # ----

        userdata["next_move"] = approach(userdata["robot_pos"], xp, yp, 0, theta_final=tp, backward=reverse)
        return 'success'
    
    
class StandbyParkEnd(yasmin.State):
    """
    SM MOVE : end of move
    """
    def __init__(self, node):
        super().__init__(outcomes=['preempted','success','fail'])

    def execute(self, userdata):
        if self.is_canceled(): return 'preempted'
        
        #TODO check that the action was actually successful ?
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'


#################################################################
#                                                               #
#                        SM STATE : MOVE                        #
#                                                               #
#################################################################

class StandbyParkSequence(Sequence):
    def __init__(self, node):
        super().__init__(
            outcomes=['preempted', 'success', 'fail'],
            states=[
                ('CALC_STANDBY_PARK_POS', MoveTo(node, CalcTargetPos())),
                ('STANDBY_PARK', Waiting(wait_time=100)),
                ('STANDBY_PARK_END', StandbyParkEnd(node))
            ]
        )

