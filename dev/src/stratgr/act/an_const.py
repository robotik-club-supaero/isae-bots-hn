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

NODE_NAME = "[ACT] "
SIMULATION = True #TODO remove this parameter

#-- GAME CONSTANTS --

READER = configparser.ConfigParser()
try :
	READER.read(os.path.join(os.path.dirname(__file__),"../../../gr_config.cfg"))
except:
	print("no file found...")

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################

## Park Position
PARKING_POS = list(literal_eval(READER.get('Robot', 'park_pos')))

## Origin Position 
ORIGIN = list(literal_eval(READER.get('Robot','start_pos')))

ROBOT_LARG = int(READER.get('Robot', 'robot_larg'))
ROBOT_LONG = int(READER.get('Robot', 'robot_long'))
ROBOT_DIAG = np.sqrt(ROBOT_LARG**2 + ROBOT_LONG**2) 

#################################################################
#                                                               #
#                       SM CONSTANTS                            #
#                                                               #
#################################################################

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

DISPLACEMENT = {
      'stop'             : -1,
      'standard'         : 0, 
      'disp_no_avoidance': 1,
      'recalage_av'      : 2,
      'recalage_ar'      : 3,
      'accurate'         : 4,
      'rotation'         : 5
}

CB_DISP = {
	-3: 'None',
	-2: 'Error Asserv',
    -1: 'Path not found',
     0: 'Disp Success',
     1: 'Path Blocked',
     2: 'Restart',
     3: 'Destination blocked'
}

#TODO : Compléter ces listes au fur et à mesure de l'avancement de l'AN

## I/O keys for states of the sm
ALL_KEY_LIST = [
    'start',
    'color',
    'score',
    'nb_actions_done',
    'cb_disp',
    'cb_arm',
    'cb_elevator',
    'cb_clamp',
    'cb_doors',
    'cb_pos',
    'arm_order',
    'depositArea',
    'nbTakeCakesError',
    'nbTakeCherriesError',
    'next_action',
    'next_pos'
    ]

ACTIONS_LIST = {
    0: 'takeCherriesPerpendicular',
    1: 'takeCherriesWall',
    2: 'depositCherries',
    3: 'takeCakes',
    4: 'depositCakes',
    5: 'park',
    6: 'preempted',
    7: 'end'
    }

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

ACTIONS_SCORE = {
	'init_score':               0, 
    'parking':                 20
}

ACTIONS_POS = {}
