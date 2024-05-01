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

import os, sys, inspect
import math
from time import sleep
import rospy
import random
from std_msgs.msg      import Int16
from geometry_msgs.msg import Pose2D
from enum import Enum

from enum import IntEnum

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
startdir = os.path.dirname(os.path.dirname(currentdir))
sys.path.insert(0,startdir)

from strat.act.an_const import DoorCallback, DoorOrder, ElevatorCallback, ElevatorOrder, \
                                    ArmCallback, ArmOrder, DspCallback

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
ARM_TIME = 0

#############################


COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

class DspOrder(IntEnum):
    STOP = 0
    MOVE_STRAIGHT = 1
    
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
        
        # Simule la reponse du BN sur l'ascenseur
        self.elevator_sub = rospy.Subscriber('/act/order/elevator', Int16, self.elevator_response)
        self.elevator_pub = rospy.Publisher("/act/callback/elevator", Int16, queue_size=10, latch=True)  

        # Simule la reponse du BN sur le bras
        self.arm_sub = rospy.Subscriber('/act/order/arm', Int16, self.arm_response)
        self.arm_pub = rospy.Publisher("/act/callback/arm", Int16, queue_size=10, latch=True)  

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
        log_info("Random Stuff generated ????")

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
        if msg.data == ElevatorOrder.MOVE_UP:
            self.elevator_pub.publish(data=ElevatorCallback.UP)
            log_info("Réponse simulée : Ascenseur haut")
        else:
            self.elevator_pub.publish(data=ElevatorCallback.DOWN)
            log_info("Réponse simulée : Ascenseur bas")

    def arm_response(self, msg):
        sleep(ARM_TIME)
        if msg.data == ArmOrder.EXTEND:
            self.arm_pub.publish(data=ArmCallback.EXTENDED)
            log_info("Réponse simulée : Bras tendu")
        else:
            self.arm_pub.publish(data=ArmCallback.RETRACTED)
            log_info("Réponse simulée : Bras rentré")

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
