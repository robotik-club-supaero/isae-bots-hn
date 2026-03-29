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

from .sm_displacement import MoveTo, approach, Approach, create_displacement_request
from strat.strat_utils import create_end_of_action_msg

from strat.strat_const import ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcParkPos(yasmin.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        super().__init__(outcomes=['preempted','success','fail'])			                   

    def execute(self, userdata):
        if self.is_canceled(): return 'preempted'

        ## Move to parking position
        x_dest, y_dest, theta = StratConfig(userdata["color"]).park_pos
        # Modif pour la strat du dernier match 

        reverse = True if userdata["color"] == 1 else False
        end_theta = 0 if userdata["color"] == 0 else 3.142

        # --- If need to go behind, go reverse as defined if angle final is close to initial
        xr, yr, tr = userdata["robot_pos"].x, userdata["robot_pos"].y, userdata["robot_pos"].theta
        opposite = ((x_dest - xr) * math.cos(theta) + (y_dest -yr) * math.sin(theta)) < 0
        delta_t = abs((theta % 3.142) - (tr % 3.142))
        if opposite:
            if not reverse:
                if (delta_t < 1.6): reverse = not reverse 
        else:
            if reverse:
                if (delta_t < 1.6): reverse = not reverse 
        # ----

        userdata["next_move"] =   create_displacement_request(x_dest, y_dest, theta=end_theta, backward=reverse) #approach(userdata["robot_pos"], x_dest, y_dest, end_theta, backward=reverse)
        return 'success'
    
    
class ParkEnd(yasmin.State):
    """
    SM PARK : Observer state
    """
    def __init__(self, node):
        super().__init__(outcomes=['preempted','success', 'fail'])

    def execute(self, userdata):
        if self.is_canceled(): return 'preempted'

        #TODO check that the action was actually successful
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

class Park(Sequence):
    def __init__(self, node):
        super().__init__(
            outcomes=['preempted', 'success', 'fail'],
            states=[
                ('CALC_PARK_POS', MoveTo(node, CalcParkPos())),
                ('PARK_END', ParkEnd(node))
            ]
        )

