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
from enum import IntEnum

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

## Config reader

READER = configparser.ConfigParser()
READER.read(os.path.join(os.path.dirname(__file__),"../../robot_config.cfg"))

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################

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
WAIT_TIME = 5
R_APPROACH_PLANTS = 300   #rayon du cercle d'approche des plantes
R_APPROACH_POTS = 200 # TODO change value
R_TAKE_POTS = 100 # TODO change value


EDGE_DIST = 20 # when turning panel
R_APPROACH_PANEL = 100 # TODO change value

SOLAR_POS = [
    2726,
    2504,
    2276,
    1729,
    1498,
    1276,
]

#################################################################
#                                                               #
#                       SM CONSTANTS                            #
#                                                               #
#################################################################

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

    
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

class Callback(IntEnum):
    @classmethod
    def parse(cls, value):
        try:
            return cls(value)
        except ValueError:
            return cls.UNKNOWN

class DspOrderMode(IntEnum):
    AVOIDANCE = 0
    STRAIGHT_NO_AVOIDANCE = 1
    STOP = 2
    BACKWARDS = 8
 
class DspCallback(Callback):
    UNKNOWN = -2
    PENDING = -3
    ERROR_ASSERV = -2
    PATH_NOT_FOUND = -1
    SUCCESS = 0
    PATH_BLOCKED = 1
    RESTART = 2 # DEPRECATED - no longer published TODO cleanup
    DESTINATION_BLOCKED = 3
     
class DoorOrder(IntEnum):
    OPEN = 0
    CLOSE = 1
    
class DoorCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    CLOSED = 0
    OPEN = 1
    
class ElevatorOrder(IntEnum):
    MOVE_UP = 1
    MOVE_DOWN = 0
        
class ElevatorCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    DOWN = 0
    UP = 1

class ClampOrder(IntEnum):
    OPEN = 0
    CLOSE = 1

class ClampCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    OPEN = 0
    CLOSED = 1

class ArmOrder(IntEnum):
    EXTEND = 1
    RETRACT = 2

class ArmCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    EXTENDED = 1
    RETRACTED = 2

class LoadDetectorCallback(Callback):
    UNKNOWN = -2
    EMPTY = 0
    LOADED = 1


## I/O keys for states of the sm
USERDATA_VAR_LIST = [ #TODO update
    'start',
    'color',
    'cb_depl',
    'cb_left_arm',
    'cb_right_arm',
    'cb_elevator',
    'cb_clamp',
    'cb_doors',
    'cb_load_detector',
    'robot_pos',
    'arm_order',
    'depositArea',
    'next_action',
    'next_move',
    'park',
    ]