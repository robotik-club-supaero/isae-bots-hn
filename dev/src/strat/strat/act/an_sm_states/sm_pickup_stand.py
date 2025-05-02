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

from ..an_const import R_APPROACH_STAND, R_TAKE_STAND
from ..an_utils import Sequence, Concurrence, OpenClamp, RiseElevator, DescendElevator, CloseClamp

from strat.strat_const import ActionResult
from strat.strat_utils import create_end_of_action_msg

from .sm_displacement import MoveTo, MoveForwardStraight, Approach, approach, approach
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
        STAND_POS = StratConfig(userdata["color"]).pickup_stand_pos

        stand_id = self._node.get_pickup_id("stand", userdata) % len(STAND_POS)

        self._msg.data = f"stand{stand_id}"
        self._node.remove_obs.publish(self._msg) # FIXME if action fails, obstacle is not restored
        
        xp, yp, tp = STAND_POS[stand_id]
        userdata["next_move"] = approach(userdata["robot_pos"], xp, yp, R_APPROACH_STAND, theta_final=tp)
             
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
        super().__init__(states=
        [
            ('CLAMP_DOWN_&_OPEN',
                Concurrence(CLAMP_1 = OpenClamp(node, etage=1), CLAMP_2 = OpenClamp(node, etage=2),
                            ELEV_1 = DescendElevator(node, etage=1), ElEV_2 = DescendElevator(node, etage=2))
            ),
            ('DEPL_TAKE_STAND', MoveForwardStraight(node, 50)),
            ('CLOSE_CLAMP', Concurrence(CLAMP_1 = CloseClamp(node, etage=1), CLAMP_2 = CloseClamp(node, etage=2))),
            ('RISE_ELEVATOR', Concurrence(ELEV_1 = RiseElevator(node, etage=1), ELEV_2 = RiseElevator(node, etage=2))),
        ]
        if etage == 1 else # etage = 2 -> bouger que le 2 !
        [
            ('CLAMP_DOWN_&_OPEN',
                Concurrence(CLAMP_2 = OpenClamp(node, etage=2),
                            ELEV_2 = DescendElevator(node, etage=2))
            ),
            ('DEPL_TAKE_STAND', MoveForwardStraight(node, 50)),
            ('CLOSE_CLAMP', CloseClamp(node, etage=2)),
            ('RISE_ELEVATOR', RiseElevator(node, etage=2)),
        ]
        )

class PickupStand(Sequence): # TODO
    def __init__(self, node, etage):
        super().__init__(states=[
            ('DEPL_POSITIONING_STAND', MoveTo(node, CalcPositionningStand(node))),
            ('PICKUP_STAND_CONC', _PickupStandSequence(node, etage)),
            ('PICKUP_STAND_END', PickupStandEnd(node.callback_action_pub)),
            ])
        self.etage = etage
    