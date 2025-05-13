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

from .sm_displacement import MoveTo, MoveForwardStraight, Approach, approach, create_displacement_request
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

        stand_pos_id = self._node.get_pickup_id("stand", userdata) % len(STAND_POS)

        self._msg.data = f"stand{stand_pos_id}"
        self._node.remove_obs.publish(self._msg) # FIXME if action fails, obstacle is not restored
        
        ((xp, yp, tp), stand_id) = STAND_POS[stand_pos_id]

        userdata["next_move"] = create_displacement_request(xp, yp, theta=tp, backward=False) #approach(userdata["robot_pos"], xp, yp, R_APPROACH_STAND, theta_final=tp)

        return 'success'
 
class PickupStandEnd(yasmin.State): # TODO
    
    def __init__(self, callback_action_pub):
        super().__init__(outcomes=['fail','success','preempted'])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        #TODO check that the action was actually successful
        self._callback_action_pub.publish(create_end_of_action_msg(exit=ActionResult.SUCCESS, reason='success'))   
        return 'success'
    
#################################################################
#                                                               #
#                        SM STATE : PICKUP_STAND                #
#                                                               #
#################################################################

class _PickupStandSequence(Sequence):
    def __init__(self, node, etage):
        super().__init__(states=[
        ('CLAMP_DOWN_&_OPEN',
            Concurrence(CLAMP_2 = OpenClamp(node, etage=etage),
                        ELEV_2 = DescendElevator(node, etage=etage))
        ),
        ('DEPL_TAKE_STAND', MoveForwardStraight(node, 100)),
        ('CLOSE_CLAMP', CloseClamp(node, etage=etage)),
        ('RISE_ELEVATOR', RiseElevator(node, etage=etage)),
        ])

class PickupStand(Sequence):
    def __init__(self, node, etage):
        super().__init__(states=[
            ('DEPL_POSITIONING_STAND', MoveTo(node, CalcPositionningStand(node))),
            ('PICKUP_STAND_CONC', _PickupStandSequence(node, etage)),
            ('PICKUP_STAND_END', PickupStandEnd(node.callback_action_pub)),
            ])
        self.etage = etage
    