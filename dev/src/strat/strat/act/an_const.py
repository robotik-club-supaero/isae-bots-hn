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

from enum import IntEnum

from config import RobotConfig, COLOR

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

## Config

CONFIG = RobotConfig()

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################

MAX_X = 2000
MAX_Y = 3000

ROBOT_LARG = CONFIG.robot_width
ROBOT_LONG = CONFIG.robot_length
ROBOT_DIAG = CONFIG.robot_diagonal

########## CONSTANTES 2024 ##########
WAIT_TIME = 5
R_APPROACH_STAND = 0
R_TAKE_STAND = 0


#################################################################
#                                                               #
#                       SM CONSTANTS                            #
#                                                               #
#################################################################
    
''' ORDERS '''

class Callback(IntEnum):
    @classmethod
    def parse(cls, value):
        try:
            return cls(value)
        except ValueError:
            return cls.UNKNOWN


class DspCallback(Callback):
    UNKNOWN = -6
    PENDING = -5
    NOT_RECOGNIZED = -4
    ERROR_ASSERV = -3     # Erreur de l'asserv (difficile à gérer)
    PATH_NOT_FOUND = -2     # La recherche de chemin n'a pas abouti
    DEST_BLOCKED = -1
    SUCCESS = 0      # Le robot est arrivé au point demandé

class ElevatorOrder(IntEnum):
    MOVE_UP = 2
    MOVE_MIDDLE = 1
    MOVE_DOWN = 0
        
class ElevatorCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    DOWN = 0
    MIDDLE = 1
    UP = 2

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

class BumperState(IntEnum):
    RELEASED = 0
    PRESSED = 1

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