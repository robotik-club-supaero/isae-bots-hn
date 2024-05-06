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

from an_const import DspOrderMode, DspCallback, R_APPROACH_POTS
from an_logging import debug_print, log_errs, log_warn, log_info
from an_utils import AutoSequence, OpenDoors, CloseDoors

from strat_const import DEPOSIT_POS
from strat_utils import adapt_pos_to_side
from an_sm_states.sm_displacement import MoveTo, MoveBackwardsStraight, Approach, colored_approach_with_angle, DISP_TIMEOUT
from an_comm import get_pickup_id, callback_action_pub, deposit_pub, disp_pub

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPots(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):    
        x, y = userdata.robot_pos[0].x, userdata.robot_pos[0].y
        pots_id = get_pickup_id("deposit pots", userdata)

        xp, yp, thetap = DEPOSIT_POS[pots_id]
        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, thetap, R_APPROACH_POTS)
             
        return 'success'

# TODO find a better way
class ReportDeposit(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success'])
    def execute(self, userdata):
        deposit_pub.publish(Empty())
        return 'success'
 
class DepositPotsEnd(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful      
        callback_action_pub.publish(exit=1, reason='success')
        
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : DEPOSIT_POTS                 #
#                                                               #
#################################################################

depositPot = AutoSequence(
    ('DEPL_POSITIONING_POTS', MoveTo(CalcPositionningPots())),
    ('OPEN_DOORS', OpenDoors()),
    ('REPORT_TO_INTERFACE', ReportDeposit()),
    ('RELEASE_POTS', MoveBackwardsStraight(dist=R_APPROACH_POTS)), # TODO change dist
    ('CLOSE_DOORS', CloseDoors()),
    ('DEPOSIT_POTS_END',  DepositPotsEnd()),
)