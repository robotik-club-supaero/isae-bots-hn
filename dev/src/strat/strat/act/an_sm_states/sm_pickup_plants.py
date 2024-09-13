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
import smach

from an_const import R_APPROACH_PLANTS
from an_utils import AutoSequence, AutoConcurrence, OpenDoors, OpenClamp, CloseDoors, CloseClamp, RiseElevator, DescendElevator, HardwareOrder

from an_sm_states.sm_displacement import MoveTo, Approach, colored_approach
from an_sm_states.sm_waiting import ObsWaitingOnce

from strat_const import PLANTS_POS, ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPlants(smach.State):
    
    def __init__(self, node):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        self._node = node
        
    def execute(self, userdata):  
        plants_id = self._node.get_pickup_id("plants", userdata)
        (xp, yp) = PLANTS_POS[plants_id]

        userdata.next_move = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.INITIAL)
                
        return 'success'
    
class CalcTakePlants(smach.State):
    
    def __init__(self, node):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        self._node = node
        
    def execute(self, userdata):              
        plants_id = self._node.get_pickup_id("plants", userdata)
        self._node.remove_obs.publish(f"plant{plants_id}") # FIXME if action fails, obstacle is not restored

        (xp, yp) = PLANTS_POS[plants_id]

        userdata.next_move = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.FINAL)

        return 'success'
       
class PickupPlantsEnd(smach.State):
    
    def __init__(self, callback_action_pub):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        self._callback_action_pub = callback_action_pub
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries plants
        self._callback_action_pub.publish(exit=ActionResult.SUCCESS, reason='success')
        
        return 'success'
        

#################################################################
#                                                               #
#                        SM STATE : PICKUP_PLANTS               #
#                                                               #
#################################################################

class _PickupPlantSequence(AutoSequence):
    def __init__(self, node):
        super().__init__(
            ('PREPARE', AutoConcurrence(
                    PREPARE_ELEVATOR = RiseElevator(node),
                    OPEN_DOORS = OpenDoors(node),
                    OPEN_CLAMP = OpenClamp(node),
                )),
            ('KEEP_OPEN', ObsWaitingOnce(wait_time=2)),
            ('TAKE', AutoConcurrence(
                CLOSE_DOORS = CloseDoors(node),
                PICKUP = AutoSequence(
                    ('DESC_ELEVATOR', DescendElevator(node)),
                    ('GRAB_PLANTS', CloseClamp(node)), # TODO check the robot has actually picked up plants
                    ('RISE_PLANTS', RiseElevator(node)),
                ),
            )),   
        ) 

class PickupPlant(AutoSequence):
    def __init__(self, node):
        super().__init__(
            ('DEPL_POSITIONING_PLANTS', MoveTo(node, CalcPositionningPlants())),             
            ('PICKUP_PLANTS_CONC', AutoConcurrence(
                DEPL_SEQ = MoveTo(node, CalcTakePlants(node)),
                PICKUP_PLANT_SEQ = _PickupPlantSequence(node),        
            )),
            ('PICKUP_PLANT_END', PickupPlantsEnd(node.callback_action_pub)),
        )