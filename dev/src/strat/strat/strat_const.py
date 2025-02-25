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
READER.read(os.path.join(os.path.dirname(__file__),'../../robot_config.cfg'))

class ActionResult(IntEnum):
    SUCCESS = 1
    NOTHING_TO_PICK_UP = 5
    FAILURE = -1

class Action(IntEnum):
    PENDING = -2
    NONE         = -1
    PICKUP_CAN = 0
    DEPOSIT_CAN = 1
    PICKUP_PLANCK   = 2
    DEPOSIT_PLANCK =  3
    CLEAR_TOP_PLANCK = 4
    PARK         = 5
    WAIT         = 6
    END          = 7


ACTIONS_OUTCOMES = {
    Action.PICKUP_CAN: 'pickupCan',
    Action.DEPOSIT_CAN: 'depositCan',
    Action.PICKUP_PLANCK: 'pickupPlanck',
    Action.DEPOSIT_PLANCK: 'depositPlanck',
    Action.CLEAR_TOP_PLANCK: 'clearTopPlanck',
    Action.PARK: 'park',
    Action.END: 'end',
    Action.WAIT: 'waiting',
}

ACTION_TRANSITIONS = {
    'pickupCan': 'PICKUP_CAN',
    'depositCan':'DEPOSIT_CAN',
    'pickupPlanck': 'PICKUP_PLANCK',
    'depositPlanck': 'DEPOSIT_PLANCK',
    'clearTopPlanck': 'CLEAR_TOP_PLANCK',
    'park':'PARK',
    'end':'END',
    'waiting':'WAITING',
}


class ActionScore(IntEnum): #TODO update
    SCORE_INIT = 0
    SCORE_PARK = 10
    SCORE_DEPOSIT_PLANCK = 4
    SCORE_COCCINELLE = 5

CAN_POS = [
    list(literal_eval(READER.get("MAP", "pickup_can_pos"))),
]

PLANCK_POS = [
    list(literal_eval(READER.get("MAP", "pickup_planck_pos"))),
]


DEPOSIT_POS = [
	list(literal_eval(READER.get("MAP", "deposit_pos"))),
]

PARK_POS = [
    list(literal_eval(READER.get("ROBOT", "init_pos"))),
    list(literal_eval(READER.get("ROBOT", "init_pos2"))),
    list(literal_eval(READER.get("ROBOT", "init_pos3")))
]