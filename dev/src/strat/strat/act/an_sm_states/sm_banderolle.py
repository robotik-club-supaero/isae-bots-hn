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
from config import StratConfig

from ..an_const import *
from ..an_utils import LaunchBanderolle, Sequence
from .sm_displacement import MoveTo, MoveBackwardsStraight, MoveForwardStraight, approach, create_displacement_request, create_orientation_request

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

        xp, yp, thetap = StratConfig(userdata["color"]).banderolle_pos
        userdata["next_move"] = create_displacement_request(xp, yp, theta=thetap, straight_only=True)

        return 'success'
    
class ReportBanderolle(yasmin.State): # DEPRECATED TODO
    def __init__(self, deposit_pub):
        super().__init__(outcomes=['success'])
        self._banderolle_pub = deposit_pub

    def execute(self, userdata):
        self._banderolle_pub.publish(Empty())
        return 'success'

class BanderolleEnd(yasmin.State):
    """
    SM PARK : Observer state
    """
    def __init__(self, node):
        super().__init__(outcomes=['preempted','success','fail'])

    def execute(self, userdata):
        if self.is_canceled():
            return 'preempted'
        
        #TODO actions before exiting the state machine

        #TODO check that the action was actually successful
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'


#################################################################
#                                                               #
#                        SM STATE : BANDEROLLE                  #
#                                                               #
#################################################################

class Banderolle(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('DEPL_POSITIONING_BANDEROLLE', MoveTo(node, CalcPosition(node))),
            ('DEPL_MOVEBACK_BANDEROLLE', MoveTo(node, MoveBackwardsStraight(node, 200))), # TODO 200 = 20 cm for now (move toward the wall)
            ('LAUNCH_BANDEROLLE', LaunchBanderolle(node)),
            ('DEPL_MOVEFORWARD_BANDEROLLE', MoveTo(node, MoveForwardStraight(node, 200))), # TODO 200 = 20 cm for now (move away from the wall)
            ('REPORT_TO_INTERFACE', ReportBanderolle(node.banderolle_pub)),
            ('BANDEROLLE_END',  BanderolleEnd(node)),
        ])

