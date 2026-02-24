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

from std_msgs.msg import Empty
from config import StratConfig, NaiveStratConfig

from ..an_const import *
from ..an_utils import CursorStickDOWN, CursorStickUP, Sequence
from .sm_displacement import MoveTo, MoveForwardStraight, MoveWithSpeed, StopRobot, PosRealign, approach, \
                             create_displacement_request, create_orientation_request

from strat.strat_utils import create_end_of_action_msg

from strat.strat_const import ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPosition(yasmin.State):

    def __init__(self, node):
        super().__init__(outcomes=['fail', 'success', 'preempted'])

    def execute(self, userdata):
        if self.is_canceled(): return 'preempted'
        
        xp, yp, thetap = StratConfig(userdata["color"]).cursor_pos
        reverse = True if userdata["color"] == 1 else False
        if reverse: thetap = (thetap + 3.142) % 6.284

        userdata["next_move"] = create_displacement_request(xp, yp, theta=thetap, straight_only=True, backward=reverse)
        return 'success'

class WaitForBumpers(yasmin.State):
    def __init__(self, logger, timeout=WAIT_TIME):
        super().__init__(outcomes=['fail', 'success', 'preempted'])
        self._timeout = timeout
        self._logger = logger

    def execute(self, userdata):
        self._logger.info(f"Waiting for the robot to touch the wall")

        begin = time.perf_counter()
        while time.perf_counter() - begin < self._timeout:
            if self.is_canceled(): return 'preempted'   
            if userdata["bumper_state"] == BumperState.PRESSED:
                return "success"
        
        self._logger.warning(f"Timeout while waiting for the bumpers")        
        return "fail"

class CursorEnd(yasmin.State):
    """
    SM CURSOR : Observer state
    """
    def __init__(self, node):
        super().__init__(outcomes=['preempted','success','fail'])

    def execute(self, userdata):
        if self.is_canceled(): return 'preempted'

        #TODO check that the action was actually successful
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'


#################################################################
#                                                                   #
#                        SM STATE : BANDEROLLE                  #
#                                                               #
#################################################################

class CursorSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('CURSOR_DEPL_POSITIONING', MoveTo(node, CalcPosition(node))),
            #('CURSOR_DEPL_MOVEBACK', MoveWithSpeed(node, -0.1, 0.)),
            #('CURSOR_WAIT_FOR_BUMPERS', WaitForBumpers(node.get_logger())),
            #('CURSOR_STOP_ROBOT', StopRobot(node)),
            #('CURSOR_REALIGN_ROBOT_POS', PosRealign(node)), # NOT TESTED !
            ('CURSOR_STICK_DOWN', CursorStickDOWN(node)),
            ('CURSOR_DEPL_MOVEFORWARD', MoveForwardStraight(node, NaiveStratConfig().cursor_distance)),
            ('CURSOR_STICK_UP', CursorStickUP(node)),
            ('CURSOR_END',  CursorEnd(node)),
        ])

