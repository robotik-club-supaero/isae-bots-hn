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

from enum import IntEnum

from config import GlobalConfig

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

## Config

CONFIG = GlobalConfig()

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################

## Origin Position 
ORIGIN_POS = CONFIG.init_zones[CONFIG.default_init_zone]

MAX_X = 2000
MAX_Y = 3000

ROBOT_LARG = CONFIG.robot_width
ROBOT_LONG = CONFIG.robot_length
ROBOT_DIAG = np.sqrt(ROBOT_LARG**2 + ROBOT_LONG**2) 

########## CONSTANTES 2024 ##########
WAIT_TIME = 5
R_APPROACH_STAND = 200 # TODO change value
R_TAKE_STAND = 100 # TODO change value


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

class BanderolleOrder(IntEnum):
    LAUNCH = 1

class BanderolleCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    STORED = 1
    LAUNCHED = 2

## I/O keys for states of the sm
USERDATA_VAR_LIST = [ #TODO update
    'start',
    'color',
    'cb_depl',
    'cb_elevator_1',
    'cb_elevator_2'
    'cb_clamp_1',
    'cb_clamp_2',
    'robot_pos',
    'depositArea',
    'next_action',
    'next_move',
    'park',
    ]