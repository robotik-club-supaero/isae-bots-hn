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
from sm_displacement import Displacement, set_next_destination

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsTakeCherriesPerpendicular(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','disp','take','redo'],
			                    input_keys=['nb_actions_done','next_pos','color','pucks_taken'],
			                    output_keys=['nb_actions_done','next_pos','pucks_taken'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        match userdata.nb_actions_done[0]:
             
            case 0:
                ## On se déplace jusqu'au site des cerises perpendiculaires au mur (en se mettant dans la bonne orientation pour le bras)
                x, y, z = ACTIONS_POS['takeCherriesPerpendicular']
                set_next_destination(userdata, x, y, z, DISPLACEMENT['standard'])
                return 'disp'

            case 1:
                ## On lance l'action de baisser le bras pour récupérer les cerises et remonter le bras.
                return 'takeCherries'

            case _:
                add_score(ACTIONS_SCORE['parking'])
                end_of_action_pub.publish(exit=1, reason='success')
                return 'done' 
                  

       


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

TakeCherriesPerpendicular = smach.StateMachine( outcomes=['preempted', 'end'],
                                                input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos', 'color','cb_arm','pucks_taken','nb_take_cherries_error'],
                                                output_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','pucks_taken','nb_take_cherries_error'])
							
with TakeCherriesPerpendicular:
	smach.StateMachine.add('OBS_TAKE_CHERRIES_PERPENDICULAR', 
                            ObsTakeCherriesPerpendicular(), 
		                    transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT'})
	smach.StateMachine.add('DISPLACEMENT', 
                            Displacement(), 
		                    transitions={'preempted':'preempted','done':'OBS_PARK','redo':'DISPLACEMENT','fail':'OBS_PARK'})

