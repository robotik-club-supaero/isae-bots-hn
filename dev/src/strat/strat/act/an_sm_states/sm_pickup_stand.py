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

from ..an_const import R_APPROACH_STAND, R_TAKE_STAND, DspOrderMode
from ..an_utils import Sequence, Concurrence, OpenClamp, RiseElevator, DescendElevator, CloseClamp

from strat.strat_const import STAND_POS, ActionResult
from strat.strat_utils import create_end_of_action_msg


from .sm_displacement import MoveTo, Approach, colored_approach_with_angle
from .sm_waiting import ObsWaitingOnce

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningStand(yasmin.State): # TODO
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._node = node
        self._msg = String()
    
    def execute(self, userdata):    
        stand_id = self._node.get_pickup_id("stand", userdata)

        self._msg.data = f"stand{stand_id}"
        self._node.remove_obs.publish(self._msg) # FIXME if action fails, obstacle is not restored
        
        xp, yp, thetap = STAND_POS[stand_id]
        userdata["next_move"] = colored_approach_with_angle(userdata["color"], xp, yp, thetap, R_APPROACH_STAND)
             
        return 'success'


class CalcTakeStand(yasmin.State): # TODO
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._node = node
        
    def execute(self, userdata):    
        pots_id = self._node.get_pickup_id("stand", userdata)

        xp, yp, thetap = POTS_POS[pots_id]
        userdata["next_move"] = colored_approach_with_angle(userdata["stand"], xp, yp, thetap, R_TAKE_STAND)
             
        return 'success'
 
class PickupStandEnd(yasmin.State): # TODO
    
    def __init__(self, callback_action_pub):
        super().__init__(outcomes=['fail','success','preempted'])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries pots
        self._callback_action_pub.publish(create_end_of_action_msg(exit=ActionResult.SUCCESS, reason='success'))   
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : PICKUP_STAND                #
#                                                               #
#################################################################

class _PickupStandSequence(Sequence): # TODO
    def __init__(self, node, etage):
        super().__init__(states=[
            ('OPEN_CLAMP', OpenClamp(node, etage)),
            ('RISE_ELEVATOR', DescendElevator(node, etage)),
            ('CLOSE_CLAMP', CloseClamp(node, etage)),
            ('RISE_ELEVATOR', RiseElevator(node, etage)),
        ])

class PickupStand(Sequence): # TODO
    def __init__(self, node, etage):
        super().__init__(states=[
            ('DEPL_POSITIONING_STAND', MoveTo(node, CalcPositionningStand(node))),
            ('PICKUP_STAND_CONC', 
                Concurrence(
                    DEPL_SEQ = MoveTo(node, CalcTakeStand(node)),
                    PICKUP_POT_SEQ = _PickupStandSequence(node, etage),
                )),
            ('PICKUP_STAND_END', PickupStandEnd(node.callback_action_pub)),
            ])
        self.etage = etage
