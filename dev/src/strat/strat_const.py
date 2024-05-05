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
import configparser, os
from ast import literal_eval


READER = configparser.ConfigParser()
READER.read(os.path.join(os.path.dirname(__file__),'../robot_config.cfg'))

class ActionResult(IntEnum):
    SUCCESS = 1
    NOTHING_TO_PICK_UP = 5
    FAILURE = -1

class Action(IntEnum):
    PENDING = -2
    NONE         = -1
    TURN_SOLAR_PANEL = 0
    PICKUP_PLANT = 1
    PICKUP_POT   = 2
    DEPOSIT_POT =  3
    PARK         = 4
    WAIT         = 5
    END          = 6
    PREEMPT      = 7


ACTIONS_LIST = [
    'turnPanel',
    'pickupPlant',
    'pickupPot',
    'depositPot',
    'park',
    'preempted',
    'end',
    'waiting',
    ]

ACTION_TRANSITIONS = {
    'turnPanel': 'TURNPANEL',
    'pickupPlant':'PICKUPPLANT',
    'pickupPot': 'PICKUPPOT',
    'depositPot': 'DEPOSITPOT',
    'park':'PARK',
    'preempted':'END',
    'end':'END',
    'waiting':'WAITING',
    }


class ActionScore(IntEnum): #TODO update
    SCORE_INIT = 0
    SCORE_PARK = 10
    SCORE_DEPOSIT_PLANTS = 4
    SCORE_SOLAR_PANEL = 5

PLANTS_POS = [
    [ 700, 1000],
    [1300, 1000],
    [1500, 1500],
    [1300, 2000],
    [ 700, 2000],
    [ 500, 1500],
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
	#[ 225,  225, -pi/2],
    [1000,  100, -pi/2],
  #  [1775,  225, -pi/2],
	[1775, 2900,  pi/2],
  #  [1000, 2775,  pi/2],
    [ 225, 2900,  pi/2],
]

PARK_POS = [
    list(literal_eval(READER.get("ROBOT", "init_pos"))),
    list(literal_eval(READER.get("ROBOT", "init_pos2"))),
    list(literal_eval(READER.get("ROBOT", "init_pos3")))
]