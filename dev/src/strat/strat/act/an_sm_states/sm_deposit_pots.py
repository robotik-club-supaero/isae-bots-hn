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

import smach
import math
import time
from geometry_msgs.msg import Quaternion
from std_msgs.msg      import Empty

from ..an_const import DspOrderMode, DspCallback, R_APPROACH_POTS
from ..an_utils import AutoSequence, OpenDoors, CloseDoors

from strat.strat_const import DEPOSIT_POS
from strat.strat_utils import adapt_pos_to_side
from .sm_displacement import MoveTo, MoveBackwardsStraight, Approach, colored_approach_with_angle, DISP_TIMEOUT

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPots(smach.State):
    
    def __init__(self, get_pickup_id):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        self._get_pickup_id = get_pickup_id
        
    def execute(self, userdata):    
        x, y = userdata.robot_pos[0].x, userdata.robot_pos[0].y
        pots_id = self._get_pickup_id("deposit pots", userdata)

        xp, yp, thetap = DEPOSIT_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_APPROACH_POTS)
             
        return 'success'

# TODO find a better way
class ReportDeposit(smach.State):
    def __init__(self, deposit_pub):
        smach.State.__init__(self, outcomes=['success'])
        self._deposit_pub = deposit_pub

    def execute(self, userdata):
        self._deposit_pub.publish(Empty())
        return 'success'
 
class DepositPotsEnd(smach.State):
    
    def __init__(self, callback_action_pub):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful      
        self._callback_action_pub.publish(exit=1, reason='success')
        
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : DEPOSIT_POTS                 #
#                                                               #
#################################################################

class DepositPot(AutoSequence):
    def __init__(self, node):
        super().__init__(
            ('DEPL_POSITIONING_POTS', MoveTo(node, CalcPositionningPots(node.get_pickup_id))),
            ('OPEN_DOORS', OpenDoors(node)),
            ('REPORT_TO_INTERFACE', ReportDeposit(node.deposit_pub)),
            ('RELEASE_POTS', MoveBackwardsStraight(node, dist=R_APPROACH_POTS)), # TODO change dist
            ('CLOSE_DOORS', CloseDoors(node)),
            ('DEPOSIT_POTS_END',  DepositPotsEnd(node.callback_action_pub)),
        )