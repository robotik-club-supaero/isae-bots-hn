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

from an_logging import debug_print
from an_const import SOLAR_POS, MAX_X, ROBOT_LONG, R_APPROACH_PANEL, EDGE_DIST, ArmCallback, ArmOrder
from an_comm import callback_action_pub, right_arm_pub, left_arm_pub, get_pickup_id
from an_utils import AutoSequence, HardwareOrder
from an_sm_states.sm_displacement import colored_approach_with_angle, MoveTo, Approach
from strat_const import ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPanel(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['color','next_action'],
                                output_keys=['next_move'])

    def execute(self, userdata):
        xp = MAX_X - ROBOT_LONG/2 - EDGE_DIST
        yp = SOLAR_POS[get_pickup_id("solar panel", userdata)]
        theta = -math.pi/2

        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, theta, R_APPROACH_PANEL)
                
        return 'success'

class ExtendArm(smach.State):
    def __init__(self):
        super().__init__(input_keys=['color', 'cb_left_arm', 'cb_right_arm'], output_keys=['cb_left_arm', 'cb_right_arm'], outcomes=['fail','success','preempted'])
        self._left = HardwareOrder(left_arm_pub, 'cb_left_arm', ArmOrder.EXTEND, ArmCallback.PENDING, ArmCallback.EXTENDED)
        self._right = HardwareOrder(right_arm_pub, 'cb_right_arm', ArmOrder.EXTEND, ArmCallback.PENDING, ArmCallback.EXTENDED)
    
    def execute(self, userdata):
        debug_print('c', "Request to extend arm")

        sm = self._left if userdata.color == 0 else self._right
        return sm.execute(userdata)

class RetractArm(smach.State):
    def __init__(self):
        super().__init__(input_keys=['color', 'cb_left_arm', 'cb_right_arm'], output_keys=['cb_left_arm', 'cb_right_arm'], outcomes=['fail','success','preempted'])
        self._left = HardwareOrder(left_arm_pub, 'cb_left_arm', ArmOrder.RETRACT, ArmCallback.PENDING, ArmCallback.RETRACTED)
        self._right = HardwareOrder(right_arm_pub, 'cb_right_arm', ArmOrder.RETRACT, ArmCallback.PENDING, ArmCallback.RETRACTED)

    def execute(self, userdata):
        debug_print('c', "Request to retract arm")
        
        sm = self._left if userdata.color == 0 else self._right
        return sm.execute(userdata)


class TurnOnePanel(AutoSequence):

    def __init__(self):
        super().__init__( 
            ("DEPL_GO_TO_PANEL", MoveTo(CalcPositionningPanel())),
            ("EXTEND_ARM", ExtendArm()),
            ("RETRACT_ARM", RetractArm()),
            ("TURN_PANEL_END", TurnPanelEnd()),
        )
    
class TurnPanelEnd(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful
        callback_action_pub.publish(exit=ActionResult.SUCCESS, reason='success')
        
        return 'success'
        
#################################################################
#                                                               #
#                 SM STATE : TURN_SOLAR_PANEL                   #
#                                                               #
#################################################################

turnPanel = TurnOnePanel()