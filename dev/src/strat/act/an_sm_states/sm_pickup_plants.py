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
import math
from numpy.linalg import norm
from an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, DspOrderMode, WAIT_TIME, R_APPROACH_PLANTS
from an_comm import callback_action_pub, add_score, doors_pub, elevator_pub
from an_utils import log_info, log_warn, log_errs, log_fatal, debug_print, debug_print_move
from geometry_msgs.msg import Quaternion, Pose2D

from an_sm_states.sm_displacement import Displacement, set_next_destination

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import PLANTS_POS

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class OpenDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_doors'],
                                output_keys=['cb_doors'])
                
        
    def execute(self, userdata):
        
        debug_print('c', "Request to open doors")
        
        userdata.cb_doors[0] = DoorCallback.PENDING
        
        doors_pub.publish(DoorOrder.OPEN.value)
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_doors[0] == DoorCallback.OPEN:
                return 'success'
            elif userdata.cb_doors[0] == DoorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_warn("Timeout")
        
        return 'fail'
    
    
class CloseDoors(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_doors'],
                                output_keys=['cb_doors'])
        
    def execute(self, userdata):
        
        debug_print('c', "Request to close doors")
                
        userdata.cb_doors[0] = DoorCallback.PENDING

        # publish close doors order
        doors_pub.publish(DoorOrder.CLOSE.value)
                
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_doors[0] == DoorCallback.CLOSED:
                return 'success'
            elif userdata.cb_doors[0] == DoorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_warn("Timeout")
        
        return 'fail'
    
class CalcPositionningPlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):
        
        '''
        xdest = xp - R/d(xp - x)
        ydest = yp - R/D(yp - y)
        '''
        
        plants_id = 2
        
        x, y = userdata.robot_pos.x, userdata.robot_pos.y
        (xp, yp) = PLANTS_POS[plants_id]
        
        d = norm([xp - x, yp - y])
        
        x_dest = xp - R_APPROACH_PLANTS/d*(xp - x)
        y_dest = yp - R_APPROACH_PLANTS/d*(yp - y)
        theta_dest = math.atan2(yp - y,xp - x)
        
        set_next_destination(userdata, x_dest, y_dest, theta_dest, DspOrderMode.AVOIDANCE)
                
        return 'success'
    
class CalcTakePlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['robot_pos','color'],
                                output_keys=['next_move'])
        
    def execute(self, userdata):
                
        '''
        xdest = xp + R/d(xp - x)
        ydest = yp + R/D(yp - y)
        '''
        
        plants_id = 2
        
        x, y = userdata.robot_pos.x, userdata.robot_pos.y
        (xp, yp) = PLANTS_POS[plants_id]
        
        d = norm([xp - x, yp - y])
        
        x_dest = xp + R_APPROACH_PLANTS/d*(xp - x)
        y_dest = yp + R_APPROACH_PLANTS/d*(yp - y)
        theta_dest = math.atan2(yp - y,xp - x)
        
        set_next_destination(userdata, x_dest, y_dest, theta_dest, DspOrderMode.AVOIDANCE)
                
        return 'success'
    
class RisePlants(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_elevator'],
                                output_keys=['cb_elevator'])
        
    def execute(self, userdata):
        
        debug_print('c', "Request to move elevator up")
        
        userdata.cb_elevator[0] = ElevatorCallback.PENDING
        
        elevator_pub.publish(ElevatorOrder.MOVE_UP.value)
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < WAIT_TIME:
            
            if userdata.cb_elevator[0] == ElevatorCallback.UP:
                return 'success'
            elif userdata.cb_elevator[0] == ElevatorCallback.BLOCKED:
                return 'fail'
            time.sleep(0.01)
            
        # timeout
        log_warn("Timeout")
        
        return 'fail'
    
    
class PickupPlantsEnd(smach.State):
    
    def __init__(self):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=[],
                                output_keys=[])
        
    def execute(self, userdata):
        
        
        #TODO check that the action was actually successful
        callback_action_pub.publish(exit=1, reason='success')
        
        return 'success'
        

#################################################################
#                                                               #
#                        SM STATE : PICKUP_PLANTS               #
#                                                               #
#################################################################

pickupPlant = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['cb_doors','next_move','robot_pos','cb_depl','next_move','nb_actions_done','cb_elevator','color'],
                                     output_keys=['cb_doors','next_move','cb_depl','nb_actions_done','cb_elevator'])
    
    
    
pickUpPlantSequence = smach.Sequence(  # sequence container
    input_keys = ['cb_doors','cb_elevator'],
    output_keys = ['cb_doors','cb_elevator'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')

deplSequence = smach.Sequence(  # sequence container
    input_keys = ['next_move','cb_depl','robot_pos','nb_actions_done','color'],
    output_keys = ['cb_depl','next_move','nb_actions_done'],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')


with pickUpPlantSequence:  # add states to the sequence #TODO define elsewhere
    smach.Sequence.add('OPEN_DOORS', OpenDoors())
    smach.Sequence.add('CLOSE_DOORS', CloseDoors())
    smach.Sequence.add('RISE_PLANTS',RisePlants())
    
with deplSequence:
    smach.Sequence.add('CALC_TAKE_PLANT', CalcTakePlants())
    smach.Sequence.add('DEPL_TAKE_PLANTS', Displacement())
    
    
pickUpPlantConcurrence = smach.Concurrence(outcomes=['success', 'fail', 'preempted'],
                            input_keys=['cb_doors','next_move','robot_pos','cb_depl','cb_elevator','nb_actions_done','color'],
                            output_keys=['cb_doors','next_move','cb_depl','cb_elevator','nb_actions_done'],
                            default_outcome='fail',
                            outcome_map={'success': { 'PICKUP_PLANT_SEQ':'success','DEPL_SEQ':'success'},
                                            'preempted' : {'PICK_UP_PLANT_SEQ':'preempted'},
                                            'preempted' : {'DEPL_SEQ' : 'preempted'}})

with pickUpPlantConcurrence:
    # Add states to the container
    smach.Concurrence.add('PICKUP_PLANT_SEQ', pickUpPlantSequence)
    smach.Concurrence.add('DEPL_SEQ', deplSequence)
    
with pickupPlant:
    smach.StateMachine.add('CALC_POSITIONING_PLANTS', CalcPositionningPlants(),
                            transitions = {'success':'DEPL_POSITIONING_PLANTS','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('DEPL_POSITIONING_PLANTS', Displacement(),
                            transitions = {'success':'PICKUP_PLANTS_CONC','fail':'fail','preempted':'preempted'})
    smach.StateMachine.add('PICKUP_PLANTS_CONC', pickUpPlantConcurrence, 
                            transitions = {'success':'PICKUP_PLANT_END', 'fail':'PICKUP_PLANT_END', 'preempted':'preempted'}
                            )
    smach.StateMachine.add('PICKUP_PLANT_END', PickupPlantsEnd(), 
                            transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'}
                            )