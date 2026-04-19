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
from ..an_const import *

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class Waiting(yasmin.State):
    """
    SM WAITING : Observer state
    """
    def __init__(self, wait_time=None, outcomes=['preempted', 'success', 'fail']):
        super().__init__(outcomes=outcomes)
        if wait_time is None:
            self._wait_time = 100
            self.predefined = False
        else:
            self._wait_time = wait_time
            self.predefined = True

    def execute(self, userdata):
        begin_time = time.time()

        if not self.predefined:
            duration = self._node.get_action_detail("wait_duration", userdata)
        else:
            duration = self._wait_time

        while time.time() - begin_time < duration:
            time.sleep(0.01)
            if self.is_canceled():
                return 'preempted'       

        return 'success'

#################################################################
#                                                               #
#                        SM STATE : WAITING                     #
#                                                               #
#################################################################

waiting = Waiting()
