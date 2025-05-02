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

from strat.act.an_const import ElevatorCallback, ElevatorOrder, \
                                    DspCallback, ClampOrder, ClampCallback, \
                                    BanderolleCallback, BanderolleOrder

from config import COLOR
from config.qos import default_profile, latch_profile, br_position_topic_profile

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

## Constants GR
ELEVATOR_TIME = 0.200
CLAMP_TIME = 0.100

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
    

        # Simule la reponse du BN sur l'ascenseur
        self.elevator_1_sub = self.create_subscription(Int16, '/act/order/elevator_1', self.elevator_1_response, default_profile)
        self.elevator_1_pub = self.create_publisher(Int16, "/act/callback/elevator_1", latch_profile)  
       
        self.elevator_2_sub = self.create_subscription(Int16, '/act/order/elevator_2', self.elevator_2_response, default_profile)
        self.elevator_2_pub = self.create_publisher(Int16, "/act/callback/elevator_2", latch_profile)  
        
        # Simule la reponse du BN sur la clamp
        self.clamp_1_sub = self.create_subscription(Int16, '/act/order/clamp_1', self.clamp_1_response, default_profile)
        self.clamp_1_pub = self.create_publisher(Int16, "/act/callback/clamp_1", latch_profile)

        self.clamp_2_sub = self.create_subscription(Int16, '/act/order/clamp_2', self.clamp_2_response, default_profile)
        self.clamp_2_pub = self.create_publisher(Int16, "/act/callback/clamp_2", latch_profile)  

        self.banderolle_sub = self.create_subscription(Int16, '/act/order/banderolle', self.banderolle_response, default_profile)
        self.banderolle_pub = self.create_publisher(Int16, "/act/callback/banderolle", latch_profile)  

    
        #### Variables ####

        self.color = 0  							# par défaut
        self.curr_pos = Position(x=0, y=0, theta=0)	# par defaut

        self.info_square = [-1] * 7  				# init undefined
        self.curr_square = 0


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

    def elevator_response(self, etage, msg):
        sleep(ELEVATOR_TIME)
        rsp = Int16()

        if msg.data == ElevatorOrder.MOVE_UP:
            rsp.data = ElevatorCallback.UP
            self.log_info(f"Réponse simulée : Ascenseur {etage} haut")
        else:
            rsp.data = ElevatorCallback.DOWN
            self.log_info(f"Réponse simulée : Ascenseur {etage} bas")

        publisher = self.elevator_1_pub if etage == 1 else self.elevator_2_pub
        publisher.publish(rsp)

    def elevator_1_response(self, msg):
        self.elevator_response(1, msg)

    def elevator_2_response(self, msg):
        self.elevator_response(2, msg)

    def clamp_response(self, etage, msg):
        sleep(CLAMP_TIME)
        rsp = Int16()

        if msg.data == ClampOrder.OPEN:
            rsp.data = ClampCallback.OPEN
            self.log_info(f"Réponse simulée : Pince {etage} ouverte")
        else:
            rsp.data = ClampCallback.CLOSED
            self.log_info(f"Réponse simulée : Pince {etage} fermée")

        publisher = self.clamp_1_pub if etage == 1 else self.clamp_2_pub
        publisher.publish(rsp)

    def clamp_1_response(self, msg):
        self.clamp_response(1, msg)

    def clamp_2_response(self, msg):
        self.clamp_response(2, msg)

    def banderolle_response(self, msg):
        sleep(BANDEROLLE_TIME)
        rsp = Int16()
        rsp.data = BanderolleCallback.LAUNCHED
        self.banderolle_pub.publish(rsp)


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
