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
    NOTHING_TO_PICK_UP = 5
    FAILURE = -1

class Action(IntEnum):
    PENDING = -2
    NONE         = -1
    PICKUP_STAND_1 = 0
    PICKUP_STAND_2 = 1
    DEPOSIT_STAND = 2
    DEPOSIT_BANDEROLLE = 3
    PARK         = 4
    WAIT         = 5
    END          = 6


ACTIONS_OUTCOMES = {
    Action.PICKUP_STAND_1: 'pickupStand_1',
    Action.PICKUP_STAND_2: 'pickupStand_2',
    Action.DEPOSIT_STAND: 'depositStand',
    Action.DEPOSIT_BANDEROLLE: 'depositBanderolle',
    Action.PARK: 'park',
    Action.END: 'end',
    Action.WAIT: 'waiting',
}

ACTION_TRANSITIONS = {
    'pickupStand_1': 'PICKUP_STAND_1',
    'pickupStand_2':'PICKUP_STAND_2',
    'depositStand': 'DEPOSIT_STAND',
    'depositBanderolle': 'DEPOSIT_BANDEROLLE',
    'park':'PARK',
    'end':'END',
    'waiting':'WAITING',
}

class ActionScore(IntEnum): #TODO update
    SCORE_INIT = 0
    SCORE_PARK = 10
    SCORE_DEPOSIT_STAND = 12
    SCORE_COCCINELLE = 5
