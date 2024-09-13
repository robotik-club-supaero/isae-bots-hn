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
import signal
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import random
from std_msgs.msg      import Int16
from geometry_msgs.msg import Pose2D
from enum import Enum

from enum import IntEnum

from strat.act.an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, \
                                    ArmCallback, ArmOrder, DspCallback, ClampOrder, ClampCallback

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

## Constants GR
DOORS_TIME = 0.200
ELEVATOR_TIME = 0.200
CLAMP_TIME = 0.100
ARM_TIME = 0.200

#############################


COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

class DspOrder(IntEnum):
    STOP = 0
    MOVE_STRAIGHT = 1
    
## ACTUATOR Node ######################################################
class ActuatorNode(Node):

    """Node used during simulations to send callbacks instead of the actuators."""

    def __init__(self):	
        super().__init__("ACN")
        self.get_logger().info("Initializing actuator node...")

        qos_profile = QoSProfile()
        latch_profile =  QoSProfile(
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Transient Local durability
        )

        #### Communication - pubs & subs ####
        # Sub a /color pour s'initialiser au set de la couleur
        self.sub_color = self.create_subscription(Int16, "/game/color", self.update_color, qos_profile)
        self.sub_pos = self.create_subscription(Pose2D,"/current_position", self.update_position, qos_profile)
    

        # Simule la reponse du BN sur les portes
        self.doors_sub = self.create_subscription(Int16, '/act/order/doors', self.doors_response, qos_profile)
        self.doors_pub = self.create_publisher(Int16, "/act/callback/doors", latch_profile)
        
        # Simule la reponse du BN sur l'ascenseur
        self.elevator_sub = self.create_subscription(Int16, '/act/order/elevator', self.elevator_response, qos_profile)
        self.elevator_pub = self.create_publisher(Int16, "/act/callback/elevator", latch_profile)  

        self.clamp_sub = self.create_subscription(Int16, '/act/order/clamp', self.clamp_response, qos_profile)
        self.clamp_pub = self.create_publisher(Int16, "/act/callback/clamp", latch_profile)  

        # Simule la reponse du BN sur le bras
        self.left_arm_sub = self.create_subscription(Int16, '/act/order/left_arm', lambda msg: ActuatorNode.arm_response(self.left_arm_pub, msg), qos_profile)
        self.left_arm_pub = self.create_publisher(Int16, "/act/callback/left_arm", latch_profile)  
        
        self.right_arm_sub = self.create_subscription(Int16, '/act/order/right_arm', lambda msg: ActuatorNode.arm_response(self.right_arm_pub, msg), qos_profile)
        self.right_arm_pub = self.create_publisher(Int16, "/act/callback/right_arm",  latch_profile)  


        # Comm avec l'interface de simulation
        self.square_layout_pub = self.create_publisher(Int16, "/simu/squareLayout", latch_profile)
        self.square_info_pub = self.create_publisher(Int16, "/simu/squareInfo", latch_profile)
        
    
        #### Variables ####

        self.color = 0  							# par défaut
        self.curr_pos = Pose2D(x=0, y=0, theta=0)	# par defaut

        self.info_square = [-1] * 7  				# init undefined
        self.curr_square = 0


        # Publication continue s'il y a besoin
        # while(True):
        # 	time.sleep(0.01)

    def log_info(self, msg):
        """Fonction intermediaire affichant les logs pendant l'execution."""
        self.get_logger().info(msg)

    ###################################################################
    # CALLBACKS
    ###################################################################

    def update_color(self, msg):
        """Callback de couleur."""
        self.color = msg.data

        # Generation aleatoire des carres de fouille
        self.log_info("Random Stuff generated ????")

    def update_position(self, msg):
        """Callback de position."""
        self.curr_pos = msg

    def doors_response(self, msg):
        sleep(DOORS_TIME)
        if msg.data == DoorOrder.OPEN:
            self.doors_pub.publish(data=DoorCallback.OPEN)
            self.log_info("Réponse simulée : Portes ouvertes")
        else:
            self.doors_pub.publish(data=DoorCallback.CLOSED)
            self.log_info("Réponse simulée : Portes fermées")

    def elevator_response(self, msg):
        sleep(ELEVATOR_TIME)
        if msg.data == ElevatorOrder.MOVE_UP:
            self.elevator_pub.publish(data=ElevatorCallback.UP)
            self.log_info("Réponse simulée : Ascenseur haut")
        else:
            self.elevator_pub.publish(data=ElevatorCallback.DOWN)
            self.log_info("Réponse simulée : Ascenseur bas")

    def clamp_response(self, msg):
        sleep(CLAMP_TIME)
        if msg.data == ClampOrder.OPEN:
            self.clamp_pub.publish(data=ClampCallback.OPEN)
            self.log_info("Réponse simulée : Pince ouverte")
        else:
            self.clamp_pub.publish(data=ClampCallback.CLOSED)
            self.log_info("Réponse simulée : Pince fermée")


    @staticmethod
    def arm_response(pub, msg):
        sleep(ARM_TIME)
        if msg.data == ArmOrder.EXTEND:
            pub.publish(data=ArmCallback.EXTENDED)
            self.log_info("Réponse simulée : Bras tendu")
        else:
            pub.publish(data=ArmCallback.RETRACTED)
            self.log_info("Réponse simulée : Bras rentré")

#################################################################
#																#
# 							Main 								#
#																#
#################################################################
def main():
    rclpy.init(args=sys.argv)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
