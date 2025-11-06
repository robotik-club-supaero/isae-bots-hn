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
import time

from std_msgs.msg import String

from config import StratConfig

from ..an_utils import Sequence, Concurrence, DrawbridgeUP, DrawbridgeDOWN, PumpsON

from strat.strat_const import ActionResult
from strat.strat_utils import create_end_of_action_msg

from .sm_displacement import MoveTo, MoveForwardStraight, Approach, approach, create_displacement_request
from .sm_waiting import ObsWaitingOnce

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionBox(yasmin.State): # TODO
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._node = node
        self._msg = String()
    
    def execute(self, userdata):    
        BOX_POS = StratConfig(userdata["color"]).pickup_boxes_pos

        box_pos_id = self._node.get_pickup_id("boxes", userdata) % len(BOX_POS)
        
        self._msg.data = f"box_{box_pos_id}"
        self._node.remove_obs.publish(self._msg) # FIXME if action fails, obstacle is not restored
        
        ((xp, yp, tp), box_id) = BOX_POS[box_pos_id]

        userdata["next_move"] = create_displacement_request(xp, yp, theta=tp, backward=False) #approach(userdata["robot_pos"], xp, yp, R_APPROACH, theta_final=tp)

        return 'success'
 
class PickupBoxEnd(yasmin.State): # TODO
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        
    def execute(self, userdata):
        #TODO check that the action was actually successful
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : DRAWBRIDGE                  #
#                                                               #
#################################################################

class _DrawbridgeSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
        ('PICKUP_PUMPS_TURN_ON', PumpsON(node)),
        ('PICKUP_DRAW_BRIDGE_DOWN', DrawbridgeDOWN(node)), # The BN manage the bumpers to stop the down / up
        ('PICKUP_DRAW_BRIDGE_UP', DrawbridgeUP(node)),
        ])

class PickupBoxesSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('PICKUP_MOVE_TO_ZONE', MoveTo(node, CalcPositionBox(node))),
            ('PICKUP_BOX_SEQ', _DrawbridgeSequence(node)),
            ('PICKUP_BOX_END', PickupBoxEnd(node)),
            ])
    