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

import time
import yasmin

from std_msgs.msg import String

from ..an_const import R_APPROACH_PLANTS
from ..an_utils import Sequence, Concurrence, OpenDoors, OpenClamp, CloseDoors, CloseClamp, RiseElevator, DescendElevator

from .sm_displacement import MoveTo, Approach, colored_approach
from .sm_waiting import ObsWaitingOnce

from strat.strat_const import PLANTS_POS, ActionResult
from strat.strat_utils import create_end_of_action_msg

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPlants(yasmin.State):
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._node = node
        
    def execute(self, userdata):  
        plants_id = self._node.get_pickup_id("plants", userdata)
        (xp, yp) = PLANTS_POS[plants_id]

        userdata["next_move"] = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.INITIAL)
                
        return 'success'
    
class CalcTakePlants(yasmin.State):
    
    def __init__(self, node):
        super().__init__(outcomes=['fail','success','preempted'])
        self._node = node
        self._msg = String()
        
    def execute(self, userdata):              
        plants_id = self._node.get_pickup_id("plants", userdata)

        self._msg.data = f"plant{plants_id}"
        self._node.remove_obs.publish(self._msg) # FIXME if action fails, obstacle is not restored

        (xp, yp) = PLANTS_POS[plants_id]

        userdata["next_move"] = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.FINAL)

        return 'success'
       
class PickupPlantsEnd(yasmin.State):
    
    def __init__(self, callback_action_pub):
        super().__init__(outcomes=['fail','success','preempted'])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries plants
        self._callback_action_pub.publish(create_end_of_action_msg(exit=ActionResult.SUCCESS, reason='success'))
        
        return 'success'
        

#################################################################
#                                                               #
#                        SM STATE : PICKUP_PLANTS               #
#                                                               #
#################################################################

class _PickupPlantSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('PREPARE', Concurrence(
                    PREPARE_ELEVATOR = RiseElevator(node),
                    OPEN_DOORS = OpenDoors(node),
                    OPEN_CLAMP = OpenClamp(node),
                )),
            ('KEEP_OPEN', ObsWaitingOnce(wait_time=2)),
            ('TAKE', Concurrence(
                CLOSE_DOORS = CloseDoors(node),
                PICKUP = Sequence(states=[
                    ('DESC_ELEVATOR', DescendElevator(node)),
                    ('GRAB_PLANTS', CloseClamp(node)), # TODO check the robot has actually picked up plants
                    ('RISE_PLANTS', RiseElevator(node)),
                ]),
            )),   
        ]) 

class PickupPlant(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('DEPL_POSITIONING_PLANTS', MoveTo(node, CalcPositionningPlants(node))),             
            ('PICKUP_PLANTS_CONC', Concurrence(
                DEPL_SEQ = MoveTo(node, CalcTakePlants(node)),
                PICKUP_PLANT_SEQ = _PickupPlantSequence(node),        
            )),
            ('PICKUP_PLANT_END', PickupPlantsEnd(node.callback_action_pub)),
        ])