#!/usr/bin/env python
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

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

NODE_NAME = "[ACT]"
SIMULATION = False if os.environ['USER'] == 'pi' else True

#-- GAME CONSTANTS --

READER = configparser.ConfigParser()
try :
	if SIMULATION: 
		#print("###simu###")
		READER.read(os.path.join(os.path.dirname(__file__),"../../../pr_start.ini"))
	else: 
		#print("###real###")
		READER.read(os.path.join(os.path.dirname(__file__),"../../../start.ini"))
except:
	print("no file found...")

PARKING_POS = list(literal_eval(READER.get('Robot', 'park_pos')))

#################################################################
#                                                               #
#                            ENUMS                              #
#                                                               #
#################################################################

class ROBOT_SIDES(IntEnum):
    HOME = 0
    AWAY = 1


## Origin postion 
ORIGIN = list(literal_eval(READER.get('Robot','start_pos')))

class DISP_ORDERS(IntEnum):
    STANDARD = 0
    STRAIGHT = 1
    RECAL_AV = 3
    RECAL_AR = 4
    ROTATION = 7
    TOUCH_AV = 12
    TOUCH_AR = 13
    ACCUR_AV = 16
    ACCUR_AR = 15


class SCORES_ACTIONS(IntEnum):
    INITIAL = 0
    PARKING = 20


class LIST_ACTIONS(IntEnum):
    NONE = -4
    PREEMPTED = -3
    PARK = -2
    STOP = -1
    WAITING = 0
    TAKE_CHERRIES_PERPENDICULAR = 1
    TAKE_CHERRIES_WALL = 2
    DEPOSIT_CHERRIES = 3
    TAKE_CAKES = 4
    DEPOSIT_CAKES = 5
    #ACTION_5 = 0
    #ACTION_6 = 0
    #ACTION_7 = 0
    #...

class CB_DISP(IntEnum):
    NONE         = -3
    ERROR_ASSERV = -2
    NOPATH_FOUND = -1
    DISP_SUCCESS = 0
    PATH_BLOCKED = 1
    DISP_RESTART = 2
    DEST_BLOCKED = 3

## I/O keys for states of the sm
ALL_KEY_LIST = [
    'start',
    'color',
    'score',
    'nb_actions_done',
    'cb_disp',
    'cb_arm',
    'cb_elevator',
    'cb_pos',
    'arm_order',
    'depositArea',
    'nbTakeGroundError',
    'nbTakeRackError',
    'next_action',
    'next_pos'
    ]

ACTIONS_LIST = [
    'takeCherriesPerpendicular',
    'takeCherriesWall',
    'depositCherries',
    'takeCakes',
    'depositCakes',
    'park',
    'preempted',
    'end',
    'waiting'
    ]

ACTIONS_STATES = {
    'takeCherriesPerpendicular':'TAKE_CHERRIES_PERPENDICULAR',
    'takeCherriesWall':'TAKE_CHERRIES_WALL',
    'depositCherries':'DEPOSIT_CHERRIES',
    'takeCakes':'TAKE_CAKES',
    'depositCakes':'DEPOSIT_CAKES',
    'park':'PARK',
    'preempted':'END',
    'end':'END',
    'waiting':'WAITING'
    }

#######################################################################
# ROBOT 
#######################################################################

ROBOT_LARG = int(READER.get('Robot', 'robot_larg'))
ROBOT_LONG = int(READER.get('Robot', 'robot_long'))
ROBOT_DIAG = np.sqrt(ROBOT_LARG**2 + ROBOT_LONG**2) 