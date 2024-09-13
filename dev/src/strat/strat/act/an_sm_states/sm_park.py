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
import smach
import math
from an_const import *
from an_sm_states.sm_displacement import Displacement, colored_approach, Approach
from strat_const import PARK_POS

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcParkPos(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self, node):
        smach.State.__init__(   self,  
                                outcomes=['preempted','success','fail'],
			                    input_keys=['cb_depl','robot_pos','next_move','color','next_action'],
			                    output_keys=['cb_depl','next_move'])
        self._node = node

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ## Move to parking position
        park_id = self._node.get_pickup_id("parking zone", userdata)
        x_dest, y_dest, theta = PARK_POS[park_id]
        # Modif pour la strat du dernier match 

        userdata.next_move = colored_approach(userdata, x_dest, y_dest, 0, Approach.INITIAL)
        return 'success'
    
    
class ParkEnd(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self, callback_action_pub):
        smach.State.__init__(   self,  
                                outcomes=['preempted','success','fail'],
			                    input_keys=[],
			                    output_keys=[])
        self._callback_action_pub = callback_action_pub

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        #TODO actions before exiting the state machine

        #TODO check that the action was actually successful
        self._callback_action_pub.publish(exit=1, reason='success')

        return 'success'


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

class Park(smach.StateMachine):
    def __init__(self, node):
        super().__init__(
            outcomes=['preempted', 'end', 'fail'],
            input_keys=['cb_depl','robot_pos','next_move', 'color', 'next_action'],
            output_keys=['cb_depl','next_move']
        )
                                
        with self:
            smach.StateMachine.add('CALC_PARK_POS', 
                                    CalcParkPos(node), 
                                    transitions={'preempted':'preempted','success':'DEPL_PARK','fail':'fail'})
            smach.StateMachine.add('DEPL_PARK', 
                                    Displacement(node), 
                                    transitions={'preempted':'preempted','success':'PARK_END','fail':'fail'})
            smach.StateMachine.add('PARK_END', 
                                    ParkEnd(node.callback_action_pub), 
                                    transitions={'preempted':'preempted','success':'end','fail':'fail'})

