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
from yasmin_viewer import YasminViewerPub
import rclpy

from std_msgs.msg      import Empty
from geometry_msgs.msg import Quaternion
from br_messages.msg import Position

# import les states de la SM
from .an_sm_states.sm_park import Park
from .an_sm_states.sm_pickup_stand import PickupStand
from .an_sm_states.sm_deposit_stand import DepositStand
from .an_sm_states.sm_banderolle import LaunchBanderolle
from .an_sm_states.sm_waiting import waiting

from .an_const import *

from ..strat_const import ACTIONS_OUTCOMES, ACTION_TRANSITIONS, ActionScore, Action
from ..strat_utils import create_quaternion

#################################################################
#                                                               #
#                       SM STATE : SETUP                        #
#                                                               #
#################################################################

class Setup(yasmin.State):
    """
    STATE MACHINE: setup the SM
    """

    def __init__(self, node):
        super().__init__(outcomes=['start', 'preempted'])
        self._node = node
        self._logger = node.get_logger()

    def execute(self, userdata):
        ##############################
        ## VARIABLES INITIALIZATION ##
        ##############################
        
        ## Game param variables
        userdata["start"] = False
        userdata["color"] = 0
        userdata["park"] = 0
        
        ## Callback of subscribers
        userdata["cb_depl"] = DspCallback.PENDING  # result of displacement action. CHECK an_const to see details on cb_depl
        userdata["robot_pos"] = Position(x=-1, y=-1, theta=-1)  # current position of the robot
        userdata["cb_clamp_1"] = ClampCallback.PENDING # state of the clamp
        userdata["cb_clamp_2"] = ClampCallback.PENDING # state of the clamp
        userdata["cb_elevator_1"] = ElevatorCallback.PENDING # state of the elevator
        userdata["cb_elevator_2"] = ElevatorCallback.PENDING # state of the elevator
        
        ## Game infos variables
        userdata["next_action"] = [Action.PENDING]  # action en cours (avec arguments eventuels)
        userdata["next_move"] = create_quaternion(x=-1, y=-1, z=-1, w=-1)

        time.sleep(0.01)
        self._node.setupComplete = True

        ##############################
        ## WAITING FOR START SIGNAL ##
        ##############################
        self._logger.info('Waiting for match to start ...')

        while not userdata["start"]:
            if self.is_canceled():
                return 'preempted'
            time.sleep(0.01)
        
        self._logger.info('Starting match !')
        return 'start'

#################################################################
#                                                               #
#                    SM STATE : REPARTITOR                      #
#                                                               #
#################################################################

class Repartitor(yasmin.State): # TO UPDATE
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self, logger, repartitor_pub):
        super().__init__(outcomes=list(ACTIONS_OUTCOMES.values())+ ["preempted"])
        self._logger = logger
        self._repartitor_pub = repartitor_pub

    def execute(self, userdata):
        self._logger.info('[Repartitor] Requesting next action ...')
        self._repartitor_pub.publish(Empty()) # demande nextAction au DN

        userdata["next_action"][0] = Action.PENDING # reset variable prochaine action
        while userdata["next_action"][0] == Action.PENDING: # en attente de reponse du DN
            if self.is_canceled():
                if userdata["next_action"][0] == Action.PENDING:
                    return 'preempted'
                break
            time.sleep(0.01)

        return ACTIONS_OUTCOMES[userdata["next_action"][0]]   # lancement prochaine action  
        
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class End(yasmin.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self, logger, disp_pub, stop_teensy_pub):
        super().__init__(outcomes=['end','preempted'])
        self._logger = logger
        self._disp_pub = disp_pub
        self._stop_teensy_pub = stop_teensy_pub

    def execute(self, userdata):
        self._logger.info('[End] Killing state machine ...')

        if self.is_canceled():
            return 'preempted'

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        self._disp_pub.publish(create_quaternion(x=0,y=0,z=0,w=-1))			# arrêt PF : w = -1
        self._stop_teensy_pub.publish(Empty())
        return 'end'

#################################################################
#                                                               #
#                        INITIALIZATION                         #
#                                                               #
#################################################################

class ActionStateMachine(yasmin.StateMachine): # TODO
    def __init__(self, node):
        super().__init__(outcomes=['exit all', 'exit preempted'])
        self._viewers = []
        self._node = node
        
        # Primary States
        self.add_state('SETUP', Setup(node), transitions={'preempted':'END','start':'REPARTITOR'})
        self.add_state('REPARTITOR', Repartitor(node.get_logger(), node.repartitor_pub),
                        transitions=dict(ACTION_TRANSITIONS, **{'preempted':'END'}))
        self.add_state('END', End(node.get_logger(), node.disp_pub, node.stop_teensy_pub),
                        transitions={'end':'exit all','preempted':'exit preempted'})
        self.add_state('WAITING', waiting,
                        transitions={'preempted':'END','success':'REPARTITOR'})

        # Specific Action States
        self.add_submachine('DEPOSIT_STAND', DepositStand(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        self.add_submachine('PICKUP_STAND_1', PickupStand(node, 1),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        self.add_submachine('PICKUP_STAND_2', PickupStand(node, 2),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        self.add_submachine('DEPOSIT_BANDEROLLE', LaunchBanderolle(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        
        # Other States
        self.add_submachine('PARK', Park(node),
                        transitions={'preempted':'END','end':'REPARTITOR','fail':'REPARTITOR'})
        

    def add_submachine(self, name, machine, transitions):
        self.add_state(name, machine, transitions)
        self._viewers.append(YasminViewerPub(name, machine, node=self._node))

    def cancel_state(self):
        # Only cancels the current submachine (i.e. the current action)
        # and keeps the global state machine active
        super().cancel_state()
        self._canceled: bool = False