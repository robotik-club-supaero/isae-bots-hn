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

import os
import numpy as np
from ast import literal_eval
import configparser
from enum import Enum, IntEnum
import rospy

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

NODE_NAME = "[ACT]"

## Config reader

READER = configparser.ConfigParser()
try :
	READER.read(os.path.join(os.path.dirname(__file__),"../../robot_config.cfg"))
except:
	rospy.log_fatal(f"{NODE_NAME} Config file not found")

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################

## Park Position
PARKING_POS = list(literal_eval(READER.get('ROBOT', 'park_pos')))

## Origin Position 
ORIGIN_POS = list(literal_eval(READER.get('ROBOT','init_pos')))

MAX_X = 2000
MAX_Y = 3000

ROBOT_LARG = int(READER.get('ROBOT', 'robot_larg'))
ROBOT_LONG = int(READER.get('ROBOT', 'robot_long'))
ROBOT_DIAG = np.sqrt(ROBOT_LARG**2 + ROBOT_LONG**2) 

DOORS_SHIFT = ROBOT_DIAG//2 + 30
ARM_SHIFT = ROBOT_DIAG//2 + 30


########## CONSTANTES 2024 ##########
WAIT_TIME = 500
R_APPROACH_PLANTS = 300   #rayon du cercle d'approche des plantes
R_APPROACH_POTS = 300 # TODO change value

#################################################################
#                                                               #
#                       SM CONSTANTS                            #
#                                                               #
#################################################################

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}


class Action(IntEnum):
    PENDING      = -2
    NONE         = -1
    PICKUP_PLANT = 0
    PARK         = 1
    WAIT         = 2
    END          = 3
    PREEMPT      = 4
    
    
''' ORDERS '''

DISPLACEMENT = {
      'standard'         : 0, 
      'noAvoidance'      : 1,
      'stop'             : 2,
      'accurate'         : 3,
      'recalage'         : 4,
      'rotation'         : 5,
      'marcheArr'        : 8
}

class DspOrderMode(IntEnum):
    AVOIDANCE = 0
    STRAIGHT_NO_AVOIDANCE = 1
    STOP = 4
    
    
class DoorOrder(Enum):
    OPEN = 0
    CLOSE = 1
    
    
    
''' CALLBACKS '''

class DspCallback(Enum):
    # UNKNOWN = -2
    # PENDING = -1
    # ARRIVED = 0
    # OBSTACLE = 1
    # OBSTACLE_ON_TARGET = 2
    # ERROR_ASSERV = 3
    
    PENDING = -3
    ERROR_ASSERV = -2
    PATH_NOT_FOUND = -1
    SUCCESS = 0
    PATH_BLOCKED = 1
    RESTART = 2
    DESTINATION_BLOCKED = 3
    
    
    # -3: 'None',
	# -2: 'Error Asserv',
    # -1: 'Path not found',
    #  0: 'Disp Success',
    #  1: 'Path Blocked',
    #  2: 'Restart',
    #  3: 'Destination blocked'
     
    
class DoorCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    CLOSED = 0
    OPEN = 1
    BLOCKED = 2
    
    
class ElevatorOrder(Enum):
    MOVE_UP = 0
    MOVE_DOWN = 1
        
class ElevatorCallback(Enum):
    UNKNOWN = -2
    PENDING = -1
    DOWN = 0
    UP = 1
    BLOCKED = 2


## I/O keys for states of the sm
USERDATA_VAR_LIST = [ #TODO update
    'start',
    'color',
    'score',
    'nb_actions_done',
    'cb_depl',
    'cb_arm',
    'cb_elevator',
    'cb_clamp',
    'cb_doors',
    'robot_pos',
    'arm_order',
    'depositArea',
    'next_action',
    'next_move',
    'deposit_area',
    'take_cakes_area',
    'take_cherries_area',
    'pucks_taken',
    'cherries_loaded',
    'error_reaction',
    'nb_errors',
    'stage_to_go',
    'stage_to_deposit',
    'park',
    'open_clamp',
    'open_doors',
    'elevator_zero'
    ]