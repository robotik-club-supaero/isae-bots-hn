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

class ActionResult(IntEnum):
    SUCCESS = 1
    NOTHING_TO_PICKUP = 5
    FAILURE = -1
    NOTHING = -2

class Action(IntEnum):
    PENDING = -2
    NONE         = -1
    PICKUP = 0
    DEPOSIT = 1
    PUSH_CURSOR = 2
    PARK         = 3
    WAIT         = 4
    END          = 5


ACTIONS_OUTCOMES = {
    Action.PICKUP: 'pickup',
    Action.DEPOSIT: 'deposit',
    Action.PUSH_CURSOR: 'push_cursor',
    Action.PARK: 'park',
    Action.END: 'end',
    Action.WAIT: 'waiting',
}

ACTION_TRANSITIONS = {
    'pickup': 'PICKUP',
    'deposit': 'DEPOSIT',
    'push_cursor': 'PUSH_CURSOR',
    'park':'PARK',
    'end':'END',
    'waiting':'WAITING',
}

class ActionScore(IntEnum): #TODO update
    SCORE_INIT = 0
    SCORE_PARK = 10
    SCORE_DEPOSIT = 4
    SCORE_PUSH_CURSOR = 10
    SCORE_PAMIS = 10
