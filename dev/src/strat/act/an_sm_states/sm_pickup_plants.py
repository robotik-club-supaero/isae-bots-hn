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

import os, sys, inspect
import time
import smach

from an_const import R_APPROACH_PLANTS
from an_comm import callback_action_pub, remove_obs, get_pickup_id
from an_logging import log_info, log_warn, log_errs, log_fatal, debug_print, debug_print_move
from an_utils import AutoSequence, AutoConcurrence, OpenDoors, OpenClamp, CloseDoors, CloseClamp, RiseElevator, DescendElevator, HardwareOrder

from an_sm_states.sm_displacement import MoveTo, Approach, colored_approach
from an_sm_states.sm_waiting import ObsWaitingOnce

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import PLANTS_POS, ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):  
        plants_id = get_pickup_id("plants", userdata)
        (xp, yp) = PLANTS_POS[plants_id]

        userdata.next_move = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.INITIAL)
                
        return 'success'
    
class CalcTakePlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color','next_action'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):              
        plants_id = get_pickup_id("plants", userdata)
        remove_obs.publish(f"plant{plants_id}") # FIXME if action fails, obstacle is not restored

        (xp, yp) = PLANTS_POS[plants_id]

        userdata.next_move = colored_approach(userdata, xp, yp, R_APPROACH_PLANTS, Approach.FINAL)

        return 'success'
       
class PickupPlantsEnd(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful
        # TODO check whether the robot actually carries plants
        callback_action_pub.publish(exit=ActionResult.SUCCESS, reason='success')
        
        return 'success'
        

#################################################################
#                                                               #
#                        SM STATE : PICKUP_PLANTS               #
#                                                               #
#################################################################

pickUpPlantSequence = AutoSequence(
    ('PREPARE', AutoConcurrence(
        PREPARE_ELEVATOR = RiseElevator(),
        OPEN_DOORS = OpenDoors(),
        OPEN_CLAMP = OpenClamp(),
    )),
    ('KEEP_OPEN', ObsWaitingOnce(wait_time=2)),
    ('TAKE', AutoConcurrence(
        CLOSE_DOORS = CloseDoors(),
        PICKUP = AutoSequence(
            ('DESC_ELEVATOR', DescendElevator()),
            ('GRAB_PLANTS', CloseClamp()), # TODO check the robot has actually picked up plants
            ('RISE_PLANTS', RiseElevator()),
        ),
    )),    
)

pickupPlant = AutoSequence(
    ('DEPL_POSITIONING_PLANTS', MoveTo(CalcPositionningPlants())),             
    ('PICKUP_PLANTS_CONC', AutoConcurrence(
        DEPL_SEQ = MoveTo(CalcTakePlants()),
        PICKUP_PLANT_SEQ = pickUpPlantSequence,        
    )),
    ('PICKUP_PLANT_END', PickupPlantsEnd()),
)