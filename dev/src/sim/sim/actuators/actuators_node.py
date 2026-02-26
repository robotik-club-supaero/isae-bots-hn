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

# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import math
import sys
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import random
from std_msgs.msg      import Int16, Empty
from br_messages.msg import Position
from enum import Enum

from enum import IntEnum

from strat.act.an_const import DspCallback, DrawbridgeOrder, DrawbridgeCallback, CursorOrder, CursorCallback, BumperState

from config import COLOR, RobotConfig
from config.qos import default_profile, latch_profile, br_position_topic_profile

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

## Constants Pour simuler le temps pris par certains actionneurs
DRAWBRIDGE_TIME = 5.
CURSOR_TIME = 1.

#############################

class DspOrder(IntEnum):
    STOP = 0
    MOVE_STRAIGHT = 1
    
## ACTUATOR Node ######################################################
class ActuatorNode(Node):
    
    """Node used during simulations to send callbacks instead of the actuators."""

    def __init__(self):	
        super().__init__("ACN")
        self.get_logger().info("Initializing actuator node...")

        #### Communication - pubs & subs ####
        # Sub a /color pour s'initialiser au set de la couleur
        self.sub_color = self.create_subscription(Int16, "/game/color", self.update_color, default_profile)
        self.sub_pos = self.create_subscription(Position,"/br/currentPosition", self.update_position, br_position_topic_profile)

        # Simule la reponse du BN sur le pont levis
        self.drawbridge_sub = self.create_subscription(Int16, '/act/order/drawbridge', self.drawbridge_response, default_profile)
        self.drawbridge_callback_pub = self.create_publisher(Int16, "/act/callback/drawbridge", latch_profile)  

        # Simule la réponse du BN sur le curseur
        self.cursor_sub = self.create_subscription(Int16, '/act/order/cursor_stick', self.cursor_response, default_profile)
        self.cursor_callback_pub = self.create_publisher(Int16, "/act/callback/cursor_stick", latch_profile)  

        self.bumpers_pub = self.create_publisher(Int16, "/act/bumpers", latch_profile)
      
        #### Variables ####

        self.color = 0  							# par défaut
        self.curr_pos = Position(x=0, y=0, theta=0)	# par defaut

        self.info_square = [-1] * 7  				# init undefined
        self.curr_square = 0

        self.update_timer = self.create_timer(0.05, self.update_bumpers)

        self.get_logger().info("Actuator node initialized")

    def log_info(self, msg):
        """Fonction intermediaire affichant les logs pendant l'execution."""
        self.get_logger().info(msg)

    ###################################################################
    # CALLBACKS
    ###################################################################

    def update_color(self, msg):
        """Callback de couleur."""
        self.color = msg.data

    def update_position(self, msg):
        """Callback de position."""
        self.curr_pos = msg

    def drawbridge_response(self, msg):
        sleep(DRAWBRIDGE_TIME)
        rsp = Int16()

        if msg.data == DrawbridgeOrder.PICKUP:
            rsp.data = DrawbridgeCallback.PICKUP
            self.log_info(f"Réponse simulée : Drawbridge PICKUP")
        elif msg.data == DrawbridgeOrder.DEPOSIT:
            rsp.data = DrawbridgeCallback.DEPOSIT
            self.log_info(f"Réponse simulée : Drawbridge DEPOSIT")
        elif msg.data == DrawbridgeOrder.STORE:
            rsp.data = DrawbridgeCallback.STORE
            self.log_info(f"Réponse simulée : Drawbridge STORE")
        else:
            rsp.data = DrawbridgeCallback.UNKNOWN
            self.log_info(f"Réponse simulée : Drawbridge UNKNOWN")

        self.drawbridge_callback_pub.publish(rsp)
    
    def cursor_response(self, msg):
        sleep(CURSOR_TIME)
        rsp = Int16()

        if msg.data == CursorOrder.DOWN:
            rsp.data = CursorCallback.DOWN
            self.log_info(f"Réponse simulée : Cursor Stick BAS")
        else:
            rsp.data = CursorCallback.UP
            self.log_info(f"Réponse simulée : Cursor Stick HAUT")

        self.cursor_callback_pub.publish(rsp)

    def update_bumpers(self):
        config = RobotConfig()
        if (self.curr_pos.x <= config.robot_length/2 and math.cos(self.curr_pos.theta) > 0.5) or \
                (self.curr_pos.x >= 2000 - config.robot_length/2 and math.cos(self.curr_pos.theta) < -0.5) or \
                (self.curr_pos.y <= config.robot_width/2 and math.sin(self.curr_pos.theta) > 0.5) or \
                (self.curr_pos.y >= 3000 - config.robot_width/2 and math.sin(self.curr_pos.theta) < -0.5):

            self.bumpers_pub.publish(Int16(data=3))

        else:
            self.bumpers_pub.publish(Int16(data=0))

#################################################################
#																#
# 							Main 								#
#																#
#################################################################
def main():
    rclpy.init(args=sys.argv)
    
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
