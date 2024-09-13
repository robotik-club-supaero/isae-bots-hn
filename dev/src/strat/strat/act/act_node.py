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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import smach
import smach_ros
import time

from std_msgs.msg import Int16, Int16MultiArray, Empty, String
from geometry_msgs.msg import Quaternion, Pose2D

from .an_const import  DoorCallback, ElevatorCallback, DspCallback, ArmCallback, LoadDetectorCallback, \
                    ClampCallback, COLOR
from .an_sm import ActionStateMachine
from .an_utils import color_dict

from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

from ..strat_const import ACTIONS_LIST, Action

class ActionNode(Node):
    def __init__(self):
        super().__init__("ACT")
        self.get_logger().info("Initializing Action Node ...")
        self.sm = None
      
        latch_profile = QoSProfile(
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Transient Local durability
        )

        """
        Initialize all publishers of AN
        """
        # GENERAL PUBS
        self.score_pub = self.create_publisher(Int16, '/game/score', latch_profile)
        self.repartitor_pub = self.create_publisher(Empty, '/strat/action/request', latch_profile)
        self.callback_action_pub = self.create_publisher(EndOfActionMsg, '/strat/action/callback', latch_profile)
        self.disp_pub = self.create_publisher(Quaternion, '/dsp/order/next_move', latch_profile)
        self.stop_teensy_pub = self.create_publisher(Quaternion, '/stop_teensy', latch_profile)
        self.remove_obs = self.create_publisher(String, '/removeObs', latch_profile)

        # SPECIFIC TO CURRENT YEAR [2024] [TODO obsolete]
        self.doors_pub = self.create_publisher(Int16, '/act/order/doors', latch_profile)
        self.elevator_pub = self.create_publisher(Int16, '/act/order/elevator', latch_profile)
        self.left_arm_pub = self.create_publisher(Int16, '/act/order/left_arm', latch_profile)
        self.right_arm_pub = self.create_publisher(Int16,'/act/order/right_arm', latch_profile)
        self.clamp_pub = self.create_publisher(Int16, '/act/order/clamp', latch_profile)
        self.deposit_pub = self.create_publisher(Empty, '/simu/deposit_end', latch_profile) # ONLY USED BY SIMU INTERFACE # TODO: use to compute score as well?

        """
        Initialize all subscribers of AN
        """
        # GENERAL SUBS
        self.start_sub = self.create_subscription(Int16, '/game/start', self.setup_start, 10)
        self.color_sub = self.create_subscription(Int16, '/game/color', self.setup_color, 10)
        self.repartitor_sub = self.create_subscription(Int16MultiArray, '/strat/action/order', self.cb_next_action, 10)
        self.disp_sub = self.create_subscription(Int16, '/dsp/callback/next_move', self.cb_depl_fct, 10)
        self.position_sub = self.create_subscription(Pose2D, '/current_position', self.cb_position_fct, 10)
        self.park_sub = self.create_subscription(Int16, '/park', self.cb_park_fct, 10)

        # SPECIFIC TO CURRENT YEAR [2024] [TODO obsolete]
        self.doors_sub = self.create_subscription(Int16, '/act/callback/doors', self.cb_doors_fct, 10)
        self.elevator_sub = self.create_subscription(Int16, '/act/callback/elevator', self.cb_elevator_fct, 10)
        self.left_arm_sub = self.create_subscription(Int16, '/act/callback/left_arm', self.cb_left_arm_fct, 10)
        self.right_arm_sub = self.create_subscription(Int16, '/act/callback/right_arm', self.cb_right_arm_fct, 10)
        self.clamp_sub = self.create_subscription(Int16, '/act/callback/clamp', self.cb_clamp_fct, 10)

        self.load_detector = self.create_subscription(Int16, '/act/callback/load_detector', self.cb_load_detector, 10) # TODO

        self.sm = ActionStateMachine(self)
        self.sis = smach_ros.IntrospectionServer('pr_an', self.sm, '/SM_ROOT')
        

    @property
    def smData(self):
        return self.sm.userdata if self.sm is not None else type("", (), {})()

    def __enter__(self):
        self.sis.start()

    def __exit__(self, *args):
        self.sis.stop()

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
        
    def debug_print_move(quaternion):
        debug_print('c*', f"({quaternion.x}, {quaternion.y}, {quaternion.z}, {quaternion.w})")

    #################################################################
    #                                                               #
    #                           FEEDBACK                            #
    #                                                               #
    #################################################################

    def setup_start(self, msg):
        """
        Callback function from topic /sm/start.
        """
        self.smData.start = (msg.data == 1)


    def setup_color(self, msg):
        """
        Callback function from topic /sm/color.
        """
        if not ok_comm : return
        if msg.data not in [0,1]:
            self.get_logger().error(f"Wrong value of color given ({msg.data})...")
            return
        else: 
            self.smData.color = msg.data
            self.get_logger().info("Received color : {}".format(COLOR[self.smData.color]))


    def cb_next_action(self, msg):
        """
        Callback for next action (DN -> Repartitor)
        """
        if msg.data[0] == -2:
            # Tmp fix from last year (= sm did not quit normally on parking...)
            self.sm.request_preempt()
            self.get_logger().info("Received stop signal (initiate parking)")
            return
        if msg.data[0] == -1:
            self.sm.request_preempt()
            self.get_logger().info("Received park signal (end of match)")
            return
        if msg.data[0] not in range(len(ACTIONS_LIST)): # Index de ACTIONS_LIST dans an_const
            self.get_logger().error(f"Wrong command from DN [/strat/repartitor] : {msg.data[0]}")
            return
        self.smData.next_action = [Action(msg.data[0])] + list(msg.data[1:])


    def cb_depl_fct(self, msg):
        """
        Callback of displacement result from Disp Node.
        """
        self.smData.cb_depl[0] = DspCallback.parse(msg.data)
        
    def cb_position_fct(self, msg):
        """
        Callback of current position of the robot.
        """
        self.smData.robot_pos[0] = msg


    def cb_doors_fct(self, msg):
        """
        Callback of the state of the doors (opened or closed)
        """
        self.smData.cb_doors[0] = DoorCallback.parse(msg.data)

    def cb_elevator_fct(self, msg):
        """
        Callback of the state of the elevator (for the cakes)
        """
        self.smData.cb_elevator[0] = ElevatorCallback.parse(msg.data)

    def cb_left_arm_fct(self, msg):
        self.smData.cb_left_arm[0] = ArmCallback.parse(msg.data)

    def cb_right_arm_fct(self, msg):
        self.smData.cb_right_arm[0] = ArmCallback.parse(msg.data)

    def cb_clamp_fct(self, msg):
        self.smData.cb_clamp[0] = ClampCallback.parse(msg.data)

    def cb_load_detector(self, msg):
        self.smData.cb_load_detector = LoadDetectorCallback.parse(msg.data)

    def cb_park_fct(self, msg):
        """
        Callback function to update sm variable XXXXX.

        <copy> this template for your update / callback functions.
        """
        self.smData.park[0] = msg.data

    def get_pickup_id(self, what, userdata):
        try:
            return userdata.next_action[1]
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
    
    node = ActionNode()

    time.sleep(1)  # NOTE : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

    with node:
        try:
            node.sm.execute()
            node.get_logger().info('Exiting state machine.')
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().warning("Node forced to terminate")
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
