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

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsWaitingOnce(smach.State):
    """
    SM WAITING : Observer state
    """
    def __init__(self, wait_time=100, outcomes=['preempted','success']):
        smach.State.__init__(self,  outcomes=outcomes)
        self._wait_time = wait_time

    def execute(self, userdata):
        begin_time = time.time()

        while time.time() - begin_time < self._wait_time:
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'       

        return 'success'
                  

class ObsWaiting(ObsWaitingOnce):
    def __init__(self, wait_time=100):
        super().__init__(wait_time, outcomes=['preempted', 'success', 'redo'])

    def execute(self, userdata):
        result = super().execute(userdata)
        if result == "success":
            return "redo"
        else:
            return result
                  

#################################################################
#                                                               #
#                        SM STATE : WAITING                     #
#                                                               #
#################################################################

waiting = smach.StateMachine(   outcomes=['preempted', 'end'],
                                input_keys=['nb_actions_done','next_move', 'color'],
                                output_keys=['nb_actions_done','next_move','color'])
							
with waiting:
    smach.StateMachine.add('OBS_WAITING', 
                            ObsWaiting(), 
                            transitions={'preempted':'preempted','success':'end','redo':'OBS_WAITING'})
   