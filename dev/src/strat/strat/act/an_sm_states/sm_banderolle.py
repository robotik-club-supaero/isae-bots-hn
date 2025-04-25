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
from .sm_displacement import Displacement, approach, Approach

from strat.strat_utils import create_end_of_action_msg

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcLaunchPos(yasmin.State):
    """
    SM LAUNCH : Observer state
    """
    def __init__(self, node):
        super().__init__(outcomes=['preempted','success','fail'])			                   
        self._node = node

    def execute(self, userdata):
        if self.is_canceled():
            return 'preempted'

        """
        ## Move to parking position
        park_id = self._node.get_pickup_id("parking zone", userdata)
        x_dest, y_dest, theta = StratConfig(userdata["color"].park_zone[park_id]
        # Modif pour la strat du dernier match 

        userdata["next_move"] = approach(user_data["robot_pos"], x_dest, y_dest, 0, Approach.INITIAL)
        """
        
        return 'success'
    
    
class BanderolleEnd(yasmin.State):
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
#                        SM STATE : BANDEROLLE                  #
#                                                               #
#################################################################

class LaunchBanderolle(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=['preempted', 'success', 'fail'])
                
        self.add_state('CALC_BANDEROLLE_POS', 
                        CalcLaunchPos(node), 
                        transitions={'preempted':'preempted','success':'DEPL_BANDEROLLE','fail':'fail'})
        self.add_state('DEPL_BANDEROLLE', 
                        Displacement(node), 
                        transitions={'preempted':'preempted','success':'BANDEROLLE_END','fail':'fail'})
        self.add_state('BANDEROLLE_END', 
                        BanderolleEnd(node.callback_action_pub), 
                        transitions={'preempted':'preempted','success':'success','fail':'fail'})

