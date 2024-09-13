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

"""
@file: obstacles_creator.py
@status: in progress

TODO: add samples as obstacles
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import numpy as np

from ast import literal_eval
from .obstacle_rect import ObstacleRect
from .obstacle_circ import ObstacleCirc
from .obstacle_tria import ObstacleTria

from ..disp_utils import *

from strat.strat_const import PLANTS_POS, POTS_POS
from strat.strat_utils import adapt_pos_to_side

HOME = 0
AWAY = 1

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def make_obstacle_list(color, logger):
    """Fonction retournant une liste d'obstacles statiques."""

    ## STATIC OBSTACLES ###############################################
    obstacles = {}

    if int(READER.get("PATHFINDER", "static_obstacles")) != 0:

        # Walls 
        obstacles["wallNorth"] = ObstacleRect(0, MARGIN, 0, 3000)
        obstacles["wallSouth"] = ObstacleRect(2000-MARGIN, 2000, 0, 3000)
        obstacles["wallEast"] = ObstacleRect(0, 2000, 0, MARGIN)
        obstacles["wallWest"] = ObstacleRect(0, 2000, 3000-MARGIN, 3000)

        # Bases
        baseHome = ObstacleRect(0, 450+MARGIN, 2550-MARGIN, 3000)
        baseAway = ObstacleRect(0, 450+MARGIN, 0, 450+MARGIN)
        if color == HOME:
            obstacles["oppBase"] = baseAway
        else:
            obstacles["oppBase"] = baseHome

        # Plants
        for i, plant in enumerate(PLANTS_POS):
            obstacles[f"plant{i}"] = ObstacleCirc(*adapt_pos_to_side(*plant, 0, color)[:2], radius=125+MARGIN)

        # Pots
        for i, pot in enumerate(POTS_POS):
            obstacles[f"pot{i}"] = ObstacleCirc(*adapt_pos_to_side(*pot, color)[:2], radius=75+MARGIN)

    logger.info("Number of static obstacles : {}.".format(len(obstacles)))
   
    return obstacles
