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

from ..an_utils import Sequence, Concurrence, DrawbridgePickup, DrawbridgeStore

from strat.strat_const import ActionResult
from strat.strat_utils import create_end_of_action_msg

from .sm_displacement import MoveTo, MoveForwardStraight, Approach, approach, create_displacement_request

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
        if self.is_canceled():
            return 'preempted'
          
        BOX_POS = StratConfig(userdata["color"]).pickup_boxes_pos

        box_pos_id = self._node.get_pickup_id("boxes", userdata) % len(BOX_POS)
        
        self._msg.data = f"box_{box_pos_id}"
        self._node.remove_obs.publish(self._msg) # FIXME if action fails, obstacle is not restored
        
        ((xp, yp, tp), box_id) = BOX_POS[box_pos_id]
        reverse = True if userdata["color"] == 1 else False
        if reverse:
            if abs(abs(tp % 3.142) - 1.571) < 0.1:  # If reverse -> only horizontal angle reversed
                tp = (tp + 3.142) % 6.284

        # --- If need to go behind, go reverse as defined if angle final is close to initial
        xr, yr, tr = userdata["robot_pos"].x, userdata["robot_pos"].y, userdata["robot_pos"].theta
        opposite = ((xp - xr) * math.cos(tr) + (yp -yr) * math.sin(tr)) < 0
        delta_t = abs((tp % 3.142) - (tr % 3.142))
        if opposite:
            if not reverse:
                if (delta_t < 1.6): reverse = not reverse 
        else:
            if reverse:
                if (delta_t < 1.6): reverse = not reverse 
        # ----

        userdata["next_move"] = create_displacement_request(xp, yp, theta=tp, backward=reverse) #approach(userdata["robot_pos"], xp, yp, R_APPROACH, theta_final=tp)

        return 'success'
 
class PickupBoxEnd(yasmin.State): # TODO
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        
    def execute(self, userdata):
        if self.is_canceled():
            return 'preempted'
        
        #TODO check that the action was actually successful
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : DRAWBRIDGE                  #
#                                                               #
#################################################################

class PickupBoxesSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('PICKUP_MOVE_TO_ZONE', MoveTo(node, CalcPositionBox(node))),
            ('PICKUP_BOX_SEQ', DrawbridgePickup(node)),
            ('PICKUP_BOX_END', PickupBoxEnd(node)),
            ])
    