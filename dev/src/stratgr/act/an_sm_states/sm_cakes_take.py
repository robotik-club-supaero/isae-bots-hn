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
from an_sm_states.sm_cakes_substates import TakePucks

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsTakeCakes(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','disp','take','redo'],
			                    input_keys=['nb_actions_done','next_pos','color','taken_area'],
			                    output_keys=['nb_actions_done','next_pos'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
            
        if userdata.nb_actions_done[0] == 0:
            ## On se déplace jusqu'au site de la pile de gâteaux visée
            x, y, z = userdata.taken_area
            set_next_destination(userdata, x, y, z, DISPLACEMENT['standard'])
            return 'disp'

        if userdata.nb_actions_done[0] == 1:
            ## On lance l'action de baisser le bras pour récupérer les cerises et remonter le bras.
            return 'deposit'

        add_score(ACTIONS_SCORE['parking'])
        end_of_action_pub.publish(exit=1, reason='success')
        return 'done' 
                  

       


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

TakeCakes = smach.StateMachine( outcomes=['preempted', 'end'],
                                input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos', 'color','cb_arm','cherries_loaded','nb_take_cherries_error'],
                                output_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','cherries_loaded','nb_take_cherries_error'])
							
with TakeCakes:
    smach.StateMachine.add('OBS_TAKE_CAKES', 
                            ObsTakeCakes(), 
                            transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT','take':'TAKE_CAKES', 'redo':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('DISPLACEMENT', 
                            Displacement(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','redo':'DISPLACEMENT','fail':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('TAKE_CAKES', 
                            TakePucks(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','fail':'OBS_TAKE_CAKES'})

