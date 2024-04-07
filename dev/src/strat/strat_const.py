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

from math import pi
from enum import IntEnum


class Action(IntEnum):
    NONE         = -1
    PICKUP_PLANT = 0
    PARK         = 1
    WAIT         = 2
    END          = 3
    PREEMPT      = 4
    

ACTIONS_LIST = [
    'pickupPlant',
    'park',
    'preempted',
    'end',
    'waiting',
    ]

ACTION_TRANSITIONS = {
    'pickupPlant':'PICKUPPLANT',
    'park':'PARK',
    'preempted':'END',
    'end':'END',
    'waiting':'WAITING',
    }


class ActionScore(IntEnum): #TODO update
    SCORE_INIT = 5
    SCORE_PARK = 15
    SCORE_DEPOSIT_PLANTS = 30
    BONUS = 20

# ACTIONS_SCORE = { 
# 	'init_score':               5,
#     'funnyCounter':            10,
#     'parking':                 15,
#     'depositStage':             1,
#     'legendary':                4,
#     'cherryOnCake':             3,
#     'cherryBucket':             1,
#     'bonus':                   20
# }


PLANTS_POS = [
    [ 700, 1000],
    [1300, 1000],
    [1600, 1500],
    [1300, 2000],
    [ 700, 2000],
    [ 400, 1500],
]

POTS_POS = [
    [ 612.5,   35, -pi/2],
    [1387.5,   35, -pi/2],
    [1965  , 1000,     0],
    [ 612.5, 2965,  pi/2],
    [1387.5, 2965,  pi/2],
    [1965  , 2000,     0],
]


DEPOSIT_POS = [
	[ 225,  225, -pi/2],
    [1000,  225, -pi/2],
    [1775,  225, -pi/2],
	[1775, 2775,  pi/2],
    [1000, 2775,  pi/2],
    [ 225, 2775,  pi/2],
]

