#!/usr/bin/env python
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
from an_const import *
from an_comm import end_of_action_pub, add_score
from an_sm_states.sm_displacement import Displacement, set_next_destination

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsPark(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','disp'],
			                    input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','color'],
			                    output_keys=['nb_actions_done','cb_disp','next_pos','cb_pos'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ## Move to parking position
        x, y, z = PARKING_POS
        if userdata.nb_actions_done[0] == 0:
            set_next_destination(userdata, x, y, z, DISPLACEMENT['standard'])
            return 'disp'

        add_score(ACTIONS_SCORE['parking'])
        end_of_action_pub.publish(exit=1, reason='success')
        return 'done'


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

Park = smach.StateMachine(  outcomes=['preempted', 'end'],
			                input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos', 'color'],
			                output_keys=['nb_actions_done','cb_disp','cb_pos','next_pos'])
							
with Park:
	smach.StateMachine.add('OBS_PARK', 
                            ObsPark(), 
		                    transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT'})
	smach.StateMachine.add('DISPLACEMENT', 
                            Displacement(), 
		                    transitions={'preempted':'preempted','done':'OBS_PARK','redo':'DISPLACEMENT','fail':'OBS_PARK'})

