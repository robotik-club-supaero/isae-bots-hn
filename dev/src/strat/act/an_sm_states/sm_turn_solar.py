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

from an_utils import debug_print
from an_const import SOLAR_POS, MAX_X, ROBOT_LONG, R_APPROACH_PANEL, EDGE_DIST, ArmCallback, ArmOrder
from an_comm import callback_action_pub, right_arm_pub as arm_pub, HardwareOrder
from an_sm_states.sm_displacement import colored_approach, colored_approach_with_angle, Displacement, Approach

CB_ARM = 'cb_right_arm'

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class CalcPositionningPanel(smach.State):
    
    def __init__(self, id):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['color','next_action'],
                                output_keys=['next_move'])
        self._panel_id = id
        
    def execute(self, userdata):
        xp = MAX_X - ROBOT_LONG/2 - EDGE_DIST
        yp = SOLAR_POS[self._panel_id]
        theta = math.pi/2

        userdata.next_move = colored_approach_with_angle(userdata.color, xp, yp, theta, R_APPROACH_PANEL)
                
        return 'success'

class ExtendArm(HardwareOrder):
    def __init__(self):
        super().__init__(arm_pub, CB_ARM, ArmOrder.EXTEND, ArmCallback.PENDING, ArmCallback.EXTENDED)

    def execute(self, userdata):
        debug_print('c', "Request to extend arm")
        return super().execute(userdata)

class RetractArm(HardwareOrder):
    def __init__(self):
        super().__init__(arm_pub, CB_ARM, ArmOrder.RETRACT, ArmCallback.PENDING, ArmCallback.RETRACTED)

    def execute(self, userdata):
        debug_print('c', "Request to retract arm")
        return super().execute(userdata)

class TurnOnePanel(smach.Sequence):

    def __init__(self, id):
        super().__init__( 
            input_keys=['color','next_move','next_action','cb_depl', CB_ARM],
            output_keys=['next_move', 'cb_depl',CB_ARM],
            outcomes =['success', 'fail', 'preempted'],
            connector_outcome = 'success'
        )
        with self:
            smach.Sequence.add("CALC_GO_TO_PANEL", CalcPositionningPanel(id))
            smach.Sequence.add("DEPL_GO_TO_PANEL", Displacement())
            smach.Sequence.add("EXTEND_ARM", ExtendArm())
            smach.Sequence.add("RETRACT_ARM", RetractArm())
    
class TurnPanelsEnd(smach.State):
    
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
#                 SM STATE : TURN_SOLAR_PANELS                  #
#                                                               #
#################################################################

turnPanels = smach.StateMachine(outcomes=['fail','success','preempted'],
                                     input_keys=['next_move','robot_pos','cb_depl','next_action','color',CB_ARM],
                                     output_keys=['next_move','cb_depl',CB_ARM])
    
turnPanelsSequence = smach.Sequence(  # sequence container
    input_keys=['next_move','robot_pos','cb_depl','next_action','color',CB_ARM],
    output_keys=['next_move','cb_depl',CB_ARM],
    outcomes = ['success', 'fail', 'preempted'],
    connector_outcome = 'success')

with turnPanelsSequence:
   smach.Sequence.add("PANEL_0", TurnOnePanel(5))
   smach.Sequence.add("PANEL_1", TurnOnePanel(4))
   smach.Sequence.add("PANEL_2", TurnOnePanel(3))
   smach.Sequence.add("PANEL_3", TurnOnePanel(2))
   smach.Sequence.add("PANEL_4", TurnOnePanel(1))
   smach.Sequence.add("PANEL_5", TurnOnePanel(0))

with turnPanels:
    smach.StateMachine.add("TURN_SEQ", turnPanelsSequence, transitions = {'success':'TURN_PANELS_END', 'fail':'fail', 'preempted':'preempted'})
    smach.StateMachine.add("TURN_PANELS_END", TurnPanelsEnd(), transitions = {'success':'success', 'fail':'fail', 'preempted':'preempted'})