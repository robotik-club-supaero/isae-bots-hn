#!/usr/bin/env python3
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
#
# pyright: reportMissingImports=false

DEBUG_PRINTS = True

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from yasmin import Blackboard
from yasmin_viewer import YasminViewerPub
import time

from std_msgs.msg import Int16, Int16MultiArray, Empty, String
from br_messages.msg import Position

from .an_const import  ElevatorCallback, DspCallback, ClampCallback, BanderolleCallback, COLOR
from .an_sm import ActionStateMachine
from .an_utils import color_dict, Color

from message.msg import EndOfActionMsg, DisplacementRequest

from ..strat_const import ACTIONS_OUTCOMES, Action
from config.qos import default_profile, latch_profile, br_position_topic_profile

class ActionNode(Node):
    def __init__(self):
        super().__init__("ACT")
        self.get_logger().info("Initializing Action Node ...")
        self.sm = None
        self.setupComplete = False
      
        """
        Initialize all publishers of /strat/action/request
        """
        # GENERAL PUBS
        self.score_pub = self.create_publisher(Int16, '/game/score', latch_profile)
        self.repartitor_pub = self.create_publisher(EndOfActionMsg, '/strat/action/request', latch_profile)
        self.disp_pub = self.create_publisher(DisplacementRequest, '/dsp/order/next_move', latch_profile)
        self.stop_teensy_pub = self.create_publisher(Empty, '/br/stop', latch_profile)
        self.remove_obs = self.create_publisher(String, '/removeObs', latch_profile)
        
        # SPECIFIC TO CURRENT YEAR [2025] [TODO obsolete]
        self.clamp_1_pub = self.create_publisher(Int16, '/act/order/clamp_1', latch_profile)
        self.clamp_2_pub = self.create_publisher(Int16, '/act/order/clamp_2', latch_profile)
        self.elevator_1_pub = self.create_publisher(Int16, '/act/order/elevator_1', latch_profile)
        self.elevator_2_pub = self.create_publisher(Int16, '/act/order/elevator_2', latch_profile)
        self.banderolle_pub = self.create_publisher(Int16, '/act/order/banderolle', latch_profile)
        self.deposit_pub = self.create_publisher(Empty, '/simu/deposit_end', latch_profile) # ONLY USED BY SIMU INTERFACE # TODO: use to compute score as well?
        """
        Initialize all subscribers of AN
        """
        # GENERAL SUBS
        self.start_sub = self.create_subscription(Int16, '/game/start', self.setup_start, default_profile)
        self.color_sub = self.create_subscription(Int16, '/game/color', self.setup_color, default_profile)
        self.repartitor_sub = self.create_subscription(Int16MultiArray, '/strat/action/order', self.cb_next_action, default_profile)
        self.disp_sub = self.create_subscription(Int16, '/dsp/callback/next_move', self.cb_depl_fct, default_profile)
        self.position_sub = self.create_subscription(Position, '/br/currentPosition', self.cb_position_fct, br_position_topic_profile)
        self.park_sub = self.create_subscription(Int16, '/park', self.cb_park_fct, default_profile)
        self.end_sub = self.create_subscription(Int16, '/game/end', self.cb_end_fct, default_profile)

        # SPECIFIC TO CURRENT YEAR [2025]
        self.clamp_1_sub = self.create_subscription(Int16, '/act/callback/clamp_1', self.cb_clamp_1_fct, default_profile)
        self.clamp_2_sub = self.create_subscription(Int16, '/act/callback/clamp_2', self.cb_clamp_2_fct, default_profile)
        self.elevator_1_sub = self.create_subscription(Int16, '/act/callback/elevator_1', self.cb_elevator_1_fct, default_profile)
        self.elevator_2_sub = self.create_subscription(Int16, '/act/callback/elevator_2', self.cb_elevator_2_fct, default_profile)
        self.banderolle_sub = self.create_subscription(Int16, '/act/callback/banderolle', self.cb_banderolle_fct, default_profile)

        self._blackboard = Blackboard()

        self.sm = ActionStateMachine(self)
        self.sis = YasminViewerPub('SM_ROOT', self.sm, node=self)
        self._sm_thread = threading.Thread(target=self.sm, args=(self._blackboard,))
        
    @property
    def smData(self):
        return self._blackboard

    def __enter__(self):
        self._sm_thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.sm.cancel_state()
        self._sm_thread.join()
        self.get_logger().info('Exiting state machine.')

    # Debug print function
    def debug_print(self, format, *msgs):
        
        # If debug prints are disabled, quit
        if not DEBUG_PRINTS: return

        # If no color was specified, error & quit
        if len(format) == 0:
            print(Color.RED + "Wrong debug_print color" + Color.RESET)
            return

        print_string = ""
        color = format[0]

        if len(format[1:]) > 0:
            shape = format[1:]
            if shape == '*': 
                print_string += Color.BOLD
            elif shape == '-': 
                print_string += Color.UNDERLINE
            elif shape == '*-': 
                print_string += Color.BOLD + Color.UNDERLINE

        total_msg = ""
        for msg in msgs:
            total_msg = total_msg + str(msg) + ", "
        total_msg = total_msg[:-2]

        try:
            print_string += color_dict[color] + total_msg + Color.RESET
            self.get_logger().info(print_string)
        except KeyError:
            self.get_logger().info(Color.RED + "Wrong debugPrint color" + Color.RESET)
            return
   
    #################################################################
    #                                                               #
    #                           FEEDBACK                            #
    #                                                               #
    #################################################################

    def setup_start(self, msg):
        """
        Callback function from topic /sm/start.
        """
        if self.setupComplete:
            self.smData["start"] = (msg.data == 1)

    def setup_color(self, msg):
        """
        Callback function from topic /sm/color.
        """
        if not self.setupComplete: return

        if msg.data not in [0,1]:
            self.get_logger().error(f"Wrong value of color given ({msg.data})...")
            return
        else: 
            self.smData["color"] = msg.data
            self.get_logger().info("Received color : {}".format(COLOR[self.smData["color"]]))

    def cb_next_action(self, msg):
        """
        Callback for next action (DN -> Repartitor)
        """
        if not self.setupComplete: return
        
        self.smData["next_action"] = [Action(msg.data[0])] + list(msg.data[1:])
        if msg.data[0] == Action.PARK:
            self.sm.cancel_state()
            self.get_logger().info("Received stop signal (initiate parking)")
            return
        if msg.data[0] == Action.END:
            self.sm.cancel_state()
            self.get_logger().info("Received park signal (end of match)")
            return
        if Action(msg.data[0]) not in ACTIONS_OUTCOMES:
            self.get_logger().error(f"Wrong command from DN [/strat/repartitor] : {msg.data[0]}")
            return
    
    def cb_depl_fct(self, msg):
        """
        Callback of displacement result from Disp Node.
        """
        if self.setupComplete:
            self.smData["cb_depl"] = DspCallback.parse(msg.data)
        
    def cb_position_fct(self, msg):
        """
        Callback of current position of the robot.
        """
        if self.setupComplete:
            self.smData["robot_pos"] = msg

    def cb_clamp_1_fct(self, msg):
        if self.setupComplete:
            self.smData["cb_clamp_1"] = ClampCallback.parse(msg.data)
    
    def cb_clamp_2_fct(self, msg):
        if self.setupComplete:
            self.smData["cb_clamp_2"] = ClampCallback.parse(msg.data)

    def cb_elevator_1_fct(self, msg):
        """
        Callback of the state of the elevator
        """
        if self.setupComplete:
            self.smData["cb_elevator_1"] = ElevatorCallback.parse(msg.data)
        
    def cb_elevator_2_fct(self, msg):
        """
        Callback of the state of the elevator
        """
        if self.setupComplete:
            self.smData["cb_elevator_2"] = ElevatorCallback.parse(msg.data)
    
    def cb_banderolle_fct(self, msg):
        """
        Callback of the state of the elevator (for the cakes)
        """
        if self.setupComplete:
            self.smData["cb_banderolle"] = BanderolleCallback.parse(msg.data)
    
    def cb_park_fct(self, msg):
        """
        Callback function to update sm variable XXXXX.

        <copy> this template for your update / callback functions.
        """
        if self.setupComplete:
            self.smData["park"] = msg.data
            if msg.data != 0:
                self.sm.cancel_state()

    def cb_end_fct(self, msg):
        self.sm.cancel_state()

    def get_pickup_id(self, what, userdata):
        try:
            return userdata["next_action"][1] # userdata["next_action"] = [Enum Action, argument optionnelle] ici l'id du truc a prendre
        except IndexError:
            self.get_logger().warning(f"No {what} id in userdata.next_action, defaulting to {what} id 0")
            return 0

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    #############################################################
    # INITIALIZATION
    #############################################################

    rclpy.init(args=sys.argv)
    
    try:
        with ActionNode() as node:
            try:
                rclpy.spin(node)
            except (ExternalShutdownException, KeyboardInterrupt):
                node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
