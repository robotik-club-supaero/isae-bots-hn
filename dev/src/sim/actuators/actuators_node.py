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
from time import sleep
import rospy
import random
from std_msgs.msg      import Int16
from geometry_msgs.msg import Pose2D

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

NODE_NAME = "[ACN]"

def log_info(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.loginfo(f"{NODE_NAME} {msg}")

def log_warn(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.log_warn(f"{NODE_NAME} {msg}")
    
def log_errs(msg):
    rospy.logerr(f"{NODE_NAME} {msg}")
    
def log_fatal(msg):
    rospy.logfatal(f"{NODE_NAME} {msg}")

## Constants GR
DOORS_TIME = 0
ELEVATOR_TIME = 0

#############################


COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

class DoorCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    CLOSED = 0
    OPEN = 1
    BLOCKED = 2
    
class DoorOrder(Enum):
    OPEN = 0
    CLOSE = 1

class DspCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    ARRIVED = 0
    OBSTACLE = 1
    OBSTACLE_ON_TARGET = 2
    ERROR_ASSERV = 3
    
class DspOrder(Enum):
    STOP = 0
    MOVE_STRAIGHT = 1
    
class ElevatorCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    DOWN = 0
    UP = 1
    BLOCKED = 2
    
## ACTUATOR Node ######################################################
class ActuatorNode():

    """Node used during simulations to send callbacks instead of the actuators."""

    def __init__(self):		

        #### Communication - pubs & subs ####
        # Sub a /color pour s'initialiser au set de la couleur
        self.sub_color = rospy.Subscriber("/game/color", Int16, self.update_color)
        self.sub_pos = rospy.Subscriber("/current_position", Pose2D, self.update_position)
    

        # Simule la reponse du BN sur les portes
        self.doors_sub = rospy.Subscriber('/act/order/doors', Int16, self.doors_response)
        self.doors_pub = rospy.Publisher("/act/callback/doors", Int16, queue_size=10, latch=True)  

        # Comm avec l'interface de simulation
        self.square_layout_pub = rospy.Publisher("/simu/squareLayout", Int16, queue_size=10, latch=True)
        self.square_info_pub = rospy.Publisher("/simu/squareInfo", Int16, queue_size=10, latch=True)
        
    
        #### Variables ####

        self.color = 0  							# par défaut
        self.curr_pos = Pose2D(x=0, y=0, theta=0)	# par defaut

        self.info_square = [-1] * 7  				# init undefined
        self.curr_square = 0


        # Publication continue s'il y a besoin
        # while(True):
        # 	time.sleep(0.01)


    ###################################################################
    # CALLBACKS
    ###################################################################

    def update_color(self, msg):
        """Callback de couleur."""
        self.color = msg.data

        # Generation aleatoire des carres de fouille
        log_info("Random generation of excavation squares.")

    def update_position(self, msg):
        """Callback de position."""
        self.curr_pos = msg

    def doors_response(self, msg):
        sleep(DOORS_TIME)
        if msg.data == DoorOrder.OPEN:
            self.doors_pub.publish(data=DoorCallback.OPEN)
            log_info("Réponse simulée : Portes ouvertes")
        else:
            self.doors_pub.publish(data=DoorCallback.CLOSED)
            log_info("Réponse simulée : Portes fermées")

    def elevator_response(self, msg):
        sleep(ELEVATOR_TIME)
        if msg.data == ElevatorCallback.UP:
            self.doors_pub.publish(data=ElevatorCallback.UP)
            log_info("Réponse simulée : Ascenseur haut")
        else:
            self.doors_pub.publish(data=ElevatorCallback.DOWN)
            log_info("Réponse simulée : Ascenseur bas")



#################################################################
#																#
# 							Main 								#
#																#
#################################################################
def main():
    rospy.init_node("ACN")
    node = ActuatorNode()
    
    log_info("Initializing Actuator Node ...")
    
    rospy.spin()


if __name__ == '__main__':
    main()
