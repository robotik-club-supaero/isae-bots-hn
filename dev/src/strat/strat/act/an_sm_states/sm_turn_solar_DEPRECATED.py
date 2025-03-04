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

import yasmin
import math

from ..an_const import SOLAR_POS, MAX_X, ROBOT_LONG, R_APPROACH_PANEL, EDGE_DIST, ArmCallback, ArmOrder
from ..an_utils import Sequence, HardwareOrder
from .sm_displacement import colored_approach_with_angle, MoveTo, Approach
from strat.strat_const import ActionResult
from strat.strat_utils import create_end_of_action_msg

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPanel(yasmin.State):
    
    def __init__(self, get_pickup_id):
        super().__init__(outcomes=['fail','success','preempted'])
        self._get_pickup_id = get_pickup_id

    def execute(self, userdata):
        xp = MAX_X - ROBOT_LONG/2 - EDGE_DIST
        yp = SOLAR_POS[self._get_pickup_id("solar panel", userdata)]
        theta = -math.pi/2

        userdata["next_move"] = colored_approach_with_angle(userdata["color"], xp, yp, theta, R_APPROACH_PANEL)
                
        return 'success'

class ExtendArm(yasmin.State):
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._left = HardwareOrder(node.get_logger(), node.left_arm_pub, 'cb_left_arm', ArmOrder.EXTEND, ArmCallback.PENDING, ArmCallback.EXTENDED)
        self._right = HardwareOrder(node.get_logger(), node.right_arm_pub, 'cb_right_arm', ArmOrder.EXTEND, ArmCallback.PENDING, ArmCallback.EXTENDED)
        self._debug_print = node.debug_print
    
    def execute(self, userdata):
        self._debug_print('c', "Request to extend arm")

        sm = self._left if userdata["color"] == 0 else self._right
        return sm.execute(userdata)

class RetractArm(yasmin.State):
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._left = HardwareOrder(node.get_logger(), node.left_arm_pub, 'cb_left_arm', ArmOrder.RETRACT, ArmCallback.PENDING, ArmCallback.RETRACTED)
        self._right = HardwareOrder(node.get_logger(), node.right_arm_pub, 'cb_right_arm', ArmOrder.RETRACT, ArmCallback.PENDING, ArmCallback.RETRACTED)
        self._debug_print = node.debug_print

    def execute(self, userdata):
        self._debug_print('c', "Request to retract arm")
        
        sm = self._left if userdata["color"] == 0 else self._right
        return sm.execute(userdata)

class TurnPanelEnd(yasmin.State):
    
    def __init__(self, callback_action_pub):
        super().__init__(outcomes=['fail','success','preempted'])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        #TODO check that the action was actually successful
        self._callback_action_pub.publish(create_end_of_action_msg(exit=ActionResult.SUCCESS, reason='success'))
        
        return 'success'
        
#################################################################
#                                                               #
#                 SM STATE : TURN_SOLAR_PANEL                   #
#                                                               #
#################################################################

class TurnPanel(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ("DEPL_GO_TO_PANEL", MoveTo(node, CalcPositionningPanel(node.get_pickup_id))),
            ("EXTEND_ARM", ExtendArm(node)),
            ("RETRACT_ARM", RetractArm(node)),
            ("TURN_PANEL_END", TurnPanelEnd(node.callback_action_pub)),
        ])