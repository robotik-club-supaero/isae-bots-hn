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

from config import COLOR

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################



########## CONSTANTES 2025 ##########
MAX_X = 2000 # Arena
MAX_Y = 3000 # Arena
WAIT_TIME = 10 # Timeout for action
from config import RobotConfig


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
    NOT_RECOGNIZED = -4 # ?
    ERROR_ASSERV = -3     # Erreur de l'asserv (difficile à gérer)
    PATH_NOT_FOUND = -2     # La recherche de chemin n'a pas abouti
    DEST_BLOCKED = -1  # Obstacle
    SUCCESS = 0      # Le robot est arrivé au point demandé

class DrawbridgeOrder(IntEnum):
    STORE = 0
    PICKUP = 1
    DEPOSIT = 2

class DrawbridgeCallback(Callback):
    UNKNOWN = -2
    PENDING = -1
    STORE = 0
    PICKUP = 1
    DEPOSIT = 2

class CursorOrder(IntEnum):
    DOWN = 0
    UP = 2

class CursorCallback(IntEnum):
    UNKNOWN = -2
    PENDING = -1
    DOWN = 2
    UP = 2

class BumperState(IntEnum):
    RELEASED = 0
    PRESSED = 1

## I/O keys for states of the sm
USERDATA_VAR_LIST = [ #TODO update
    'start',
    'color',
    'cb_depl',
    'cb_drawbridge',
    'cb_cursor_stick',
    'bumper_state',
    'robot_pos',
    'robot_pos_realignment',
    'depositArea',
    'next_action',
    'next_move',
    'park',
    ]