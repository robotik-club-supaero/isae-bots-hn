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
import threading

from std_msgs.msg      import Empty
from br_messages.msg import Position

# import les states de la SM
from .an_sm_states.sm_park import Park
from .an_sm_states.sm_cursor import CursorSequence
from .an_sm_states.sm_deposit_box import DepositBoxesSequence
from .an_sm_states.sm_pickup_boxes import PickupBoxesSequence
from .an_sm_states.sm_waiting import waiting, Waiting
from .an_sm_states.sm_standby_park import StandbyParkSequence
from .an_sm_states.sm_displacement import create_displacement_request, create_stop_BR_request

from message.msg import EndOfActionMsg
from std_msgs.msg import Int16MultiArray
from config.qos import default_profile

from .an_const import *

from ..strat_const import ACTIONS_OUTCOMES, ACTION_TRANSITIONS, ActionScore, Action, ActionResult

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
        userdata["end"] = False
        userdata["color"] = 0
        userdata["go_park"] = 0
        
        ## Callback of subscribers
        userdata["cb_depl"] = DspCallback.PENDING  # result of displacement action. CHECK an_const to see details on cb_depl
        userdata["robot_pos"] = Position(x=-1, y=-1, theta=-1)  # current position of the robot
        userdata["robot_pos_realignement"] = Position(x=0, y=0, theta=0)  # current position realignement of the robot
        userdata["cb_drawbridge"] = DrawbridgeCallback.PENDING # state of the clamp
        userdata["cb_cursor_stick"] = CursorCallback.PENDING # state of the elevator
        userdata["cb_pumps"] = PumpsCallback.PENDING # state of the pumps
        userdata["bumper_state"] = BumperState.RELEASED
        
        ## Game infos variables
        userdata["next_action"] = [Action.PICKUP, 2]  # action en cours (avec arguments eventuels)
        userdata["next_move"] = create_displacement_request(x=-1, y=-1)

        userdata["action_result"] = ActionResult.NOTHING

        self._node.reset_act_pub.publish(Empty())
        time.sleep(0.01)
        self._node.setupComplete = True

        ##############################
        ## WAITING FOR START SIGNAL ##
        ##############################
        self._logger.info('Waiting for match to start ...')

        while not userdata["start"]:
            if self.is_canceled(): return 'preempted'
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
        super().__init__(outcomes=list(ACTIONS_OUTCOMES.values()) + ["preempted"])
        self._logger = logger
        self._repartitor_pub = repartitor_pub

    def execute(self, userdata):
        if self.is_canceled(): return "preempted"

        if userdata["end"]:
            self._logger.info('[Repartitor] Ended.')
            return 'end'
        
        self._logger.info('[Repartitor] Requesting next action ...')

        self._repartitor_pub.publish(EndOfActionMsg(exit=userdata["action_result"])) # demande nextAction au DN

        userdata["action_result"] = ActionResult.NOTHING # Reset pour la prochaine action
        userdata["next_action"] = [Action.PENDING] # reset variable prochaine action
        
        while userdata["next_action"][0] == Action.PENDING: # en attente de reponse du DN
            time.sleep(0.1)
            if self.is_canceled(): return "preempted"

        self._logger.info(f'[Repartitor] Repartitor has received the next action : {userdata["next_action"]} -> {ACTIONS_OUTCOMES[userdata["next_action"][0]]}')

        return ACTIONS_OUTCOMES[userdata["next_action"][0]]   # lancement prochaine action
        
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class End(yasmin.State):
    """
    STATE MACHINE : Last action to stop the robot
    """

    def __init__(self, logger, disp_pub, stop_teensy_pub):
        super().__init__(outcomes=['end', 'preempted'])
        self._logger = logger
        self._disp_pub = disp_pub
        self._stop_teensy_pub = stop_teensy_pub

    def execute(self, userdata):
        if self.is_canceled(): return "preempted"
        self._logger.info('[End] Killing state machine ...')

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        self._disp_pub.publish(create_stop_BR_request())
        self._stop_teensy_pub.publish(Empty())
        return 'end'

#################################################################
#                                                               #
#                        INITIALIZATION                         #
#                                                               #
#################################################################

class ActionStateMachine(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=['exit all', 'exit preempted'])
        self._viewers = []
        self._node = node
        self._force_transition_repartitor = False
        
        # Primary States
        self.add_state('SETUP', Setup(node), transitions={'preempted':'END','start':'REPARTITOR'})
        self.add_state('REPARTITOR', Repartitor(node.get_logger(), node.repartitor_pub),
                        transitions=dict(ACTION_TRANSITIONS, **{'preempted':'REPARTITOR'}))
        
        self.add_state('END', End(node.get_logger(), node.disp_pub, node.stop_teensy_pub),
                        transitions={'end':'exit all', 'preempted': 'END'})
        self.add_state('WAIT', waiting,
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        
        # Specific Action States
        self.add_submachine('DEPOSIT', DepositBoxesSequence(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        self.add_submachine('PICKUP', PickupBoxesSequence(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        self.add_submachine('CURSOR', CursorSequence(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        
        # Other States
        self.add_submachine('PARKSTANDBY', StandbyParkSequence(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
        self.add_submachine('PARK', Park(node),
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'REPARTITOR'})
             
    
    def add_submachine(self, name, machine, transitions):
        self.add_state(name, machine, transitions)
        #self._viewers.append(YasminViewerPub(name, machine, node=self._node))
    
    def cancel_state(self):
        # Only cancel the current child action, NOT the top-level SM itself.
        # In YASMIN 3.5.1, status is tracked via StateStatus enum (not a _canceled bool),
        # so the old "self._canceled = False" reset was a no-op — the SM stayed CANCELED,
        # causing a RuntimeError when the execute loop tried to transition to REPARTITOR.
        self._node.get_logger().info("Canceling state...")
        # OLD (broken — self._canceled = False had no effect in YASMIN 3.5.1):
        # super().cancel_state()
        # self._canceled = False
        if self.is_running():
            current_state_name = self.get_current_state()
            while not current_state_name:
                time.sleep(0.001)
                current_state_name = self.get_current_state()
            if current_state_name and current_state_name in self.get_states():
                self.get_states()[current_state_name]["state"].cancel_state()

    def cancel_all(self):
        """Full cancel for node shutdown — stops the SM thread completely."""
        self._node.get_logger().info("Full shutdown cancel...")
        super().cancel_state()
