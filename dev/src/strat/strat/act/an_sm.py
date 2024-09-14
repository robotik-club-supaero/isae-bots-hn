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
import rclpy

from std_msgs.msg      import Empty
from geometry_msgs.msg import Quaternion

# import les states de la SM
from .an_sm_states.sm_park import Park
from .an_sm_states.sm_turn_solar import TurnPanel
from .an_sm_states.sm_pickup_plants import PickupPlant
from .an_sm_states.sm_pickup_pots import PickupPlot
from .an_sm_states.sm_deposit_pots import DepositPot
from .an_sm_states.sm_waiting import waiting

from .an_const import *

from geometry_msgs.msg import Quaternion, Pose2D

from ..strat_const import ACTIONS_LIST, ACTION_TRANSITIONS, ActionScore, Action
from ..strat_utils import create_quaternion

#################################################################
#                                                               #
#                       SM STATE : SETUP                        #
#                                                               #
#################################################################

class Setup(smach.State):
    """
    STATE MACHINE: setup the SM
    """

    def __init__(self, logger):
        smach.State.__init__(	self, 	
                                outcomes=['start', 'preempted'],
                                input_keys=USERDATA_VAR_LIST,
                                output_keys=USERDATA_VAR_LIST)
        self._logger = logger

    def execute(self, userdata):
        ##############################
        ## VARIABLES INITIALIZATION ##
        ##############################
        
        ## Game param variables
        userdata.start = False
        userdata.color = 0
        userdata.park = [0] 
        
        ## Callback of subscribers
        userdata.cb_depl = [DspCallback.PENDING]  # result of displacement action. CHECK an_const to see details on cb_depl
        userdata.robot_pos = [Pose2D(x=-1, y=-1, theta=-1)]  # current position of the robot
        userdata.cb_left_arm = [-1]    # state of the arm
        userdata.cb_right_arm = [-1]    # state of the arm
        userdata.cb_doors = [-1]	# state of the doors
        userdata.cb_clamp = [-1] 	# state of the clamp
        userdata.cb_elevator = [-1] # state of the elevator
        userdata.cb_load_detector = [LoadDetectorCallback.EMPTY] # whether the robot carries something # TODO no hardware to detect it

        ## Game infos variables
        userdata.next_action = [Action.PENDING]  # action en cours (avec arguments eventuels)
        userdata.next_move = create_quaternion(x=-1, y=-1, z=-1, w=-1)

        time.sleep(0.01)

        ##############################
        ## WAITING FOR START SIGNAL ##
        ##############################
        self._logger.info('Waiting for match to start ...')

        while not userdata.start:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(0.01)
        
        self._logger.info('Starting match !')
        return 'start'

#################################################################
#                                                               #
#                    SM STATE : REPARTITOR                      #
#                                                               #
#################################################################

class Repartitor(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self, logger, repartitor_pub):
        smach.State.__init__(	self, 	
                                outcomes=ACTIONS_LIST,
                                input_keys=['next_action'],
                                output_keys=['next_action'])
        self._logger = logger
        self._repartitor_pub = repartitor_pub

    def execute(self, userdata):
        self._logger.info('[Repartitor] Requesting next action ...')
        self._repartitor_pub.publish(Empty()) # demande nextAction au DN

        userdata.next_action[0] = Action.PENDING # reset variable prochaine action
        while userdata.next_action[0] == Action.PENDING: # en attente de reponse du DN
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(0.01)

        return ACTIONS_LIST[userdata.next_action[0].value]   # lancement prochaine action  
        
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class End(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self, logger, disp_pub, stop_teensy_pub):
        smach.State.__init__(	self, 	
                                 outcomes=['end','preempted'],
                                input_keys=[''],
                                output_keys=[''])
        self._logger = logger
        self._disp_pub = disp_pub
        self._stop_teensy_pub = stop_teensy_pub

    def execute(self, userdata):
        self._logger.info('[End] Killing state machine ...')

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        self._disp_pub.publish(create_quaternion(x=0,y=0,z=0,w=-1))			# arrêt PF : w = -1
        self._stop_teensy_pub.publish(create_quaternion(x=0,y=0,z=0,w=2))	# arrêt BR (code w=2 pour le BN)
        return 'end'

#################################################################
#                                                               #
#                        INITIALIZATION                         #
#                                                               #
#################################################################

class ActionStateMachine(smach.StateMachine):
    def __init__(self, node):
        smach.StateMachine.__init__(self,
            outcomes=['exit all', 'exit preempted'],
            input_keys=USERDATA_VAR_LIST,
            output_keys=USERDATA_VAR_LIST
        )

        with self:
            # Primary States
            smach.StateMachine.add('SETUP', 
                                    Setup(node.get_logger()),
                                    transitions={'preempted':'END','start':'REPARTITOR'})
            smach.StateMachine.add('REPARTITOR', 
                                    Repartitor(node.get_logger(), node.repartitor_pub),
                                    transitions=ACTION_TRANSITIONS)
            smach.StateMachine.add('END', 
                                    End(node.get_logger(), node.disp_pub, node.stop_teensy_pub),
                                    transitions={'end':'exit all','preempted':'exit preempted'})

            # Specific Action States
            smach.StateMachine.add('TURNPANEL', TurnPanel(node),
                            transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})

            smach.StateMachine.add('PICKUPPLANT', PickupPlant(node),
                            transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})
            smach.StateMachine.add('PICKUPPOT', PickupPlot(node),
                            transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})
            smach.StateMachine.add('DEPOSITPOT', DepositPot(node),
                            transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})
    
            # Other States
            smach.StateMachine.add('PARK', Park(node),
                                    transitions={'preempted':'END','end':'REPARTITOR','fail':'REPARTITOR'})
            smach.StateMachine.add('WAITING', waiting,
                                    transitions={'preempted':'END','success':'REPARTITOR'})