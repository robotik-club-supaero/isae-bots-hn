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


#TODO : Compléter ces listes au fur et à mesure de l'avancement de l'AN

## I/O keys for states of the sm
ALL_KEY_LIST = [ #TODO update
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
    'next_action',
    'next_pos',
    'deposit_area',
    'take_cakes_area',
    'take_cherries_area',
    'pucks_taken',
    'cherries_loaded',
    'error_reaction',
    'nb_errors',
    'stage_to_go',
    'stage_to_deposit',
    'backward',
    'park',
    'open_clamp',
    'open_doors',
    'elevator_zero'
    ]

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

ACTIONS_SCORE = { #TODO update
	'init_score':               5,
    'funnyCounter':            10,
    'parking':                 15,
    'depositStage':             1,
    'legendary':                4,
    'cherryOnCake':             3,
    'cherryBucket':             1,
    'bonus':                   20
}


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


DEPOSIT_POS = {
	[ 225,  225, -pi/2],
    [1000,  225, -pi/2],
    [1775,  225, -pi/2],
	[1775, 2775,  pi/2],
    [1000, 2775,  pi/2],
    [ 225, 2775,  pi/2],
}

