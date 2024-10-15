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
from ..an_const import *
from .sm_displacement import Displacement, colored_approach, Approach
from strat.strat_const import PARK_POS
from strat.strat_utils import create_end_of_action_msg

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcParkPos(yasmin.State):
    """
    SM PARK : Observer state
    """
    def __init__(self, node):
        super().__init__(outcomes=['preempted','success','fail'])			                   
        self._node = node

    def execute(self, userdata):
        if self.is_canceled():
            return 'preempted'

        ## Move to parking position
        park_id = self._node.get_pickup_id("parking zone", userdata)
        x_dest, y_dest, theta = PARK_POS[park_id]
        # Modif pour la strat du dernier match 

        userdata["next_move"] = colored_approach(userdata, x_dest, y_dest, 0, Approach.INITIAL)
        return 'success'
    
    
class ParkEnd(yasmin.State):
    """
    SM PARK : Observer state
    """
    def __init__(self, callback_action_pub):
        super().__init__(outcomes=['preempted','success','fail'])
        self._callback_action_pub = callback_action_pub

    def execute(self, userdata):
        if self.is_canceled():
            return 'preempted'
        
        #TODO actions before exiting the state machine

        #TODO check that the action was actually successful
        self._callback_action_pub.publish(create_end_of_action_msg(exit=1, reason='success'))

        return 'success'


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

class Park(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=['preempted', 'end', 'fail'])
                
        self.add_state('CALC_PARK_POS', 
                        CalcParkPos(node), 
                        transitions={'preempted':'preempted','success':'DEPL_PARK','fail':'fail'})
        self.add_state('DEPL_PARK', 
                        Displacement(node), 
                        transitions={'preempted':'preempted','success':'PARK_END','fail':'fail'})
        self.add_state('PARK_END', 
                        ParkEnd(node.callback_action_pub), 
                        transitions={'preempted':'preempted','success':'end','fail':'fail'})

