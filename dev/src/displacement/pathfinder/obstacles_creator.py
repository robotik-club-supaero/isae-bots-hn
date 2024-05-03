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
import os, sys, inspect

from ast import literal_eval
from pathfinder.obstacle_rect import ObstacleRect
from pathfinder.obstacle_circ import ObstacleCirc
from pathfinder.obstacle_tria import ObstacleTria

from disp_utils import *


#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
startdir = os.path.dirname(os.path.dirname(currentdir))
sys.path.insert(0,startdir)

from strat.strat_const import PLANTS_POS, POTS_POS

HOME = 0
AWAY = 1

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def make_obstacle_list(color):
    """Fonction retournant une liste d'obstacles statiques."""

    ## STATIC OBSTACLES ###############################################
    obstacles = {}

    if int(READER.get("PATHFINDER", "static_obstacles")) != 0:
   
        # Walls 
        obstacles["wallNorth"] = ObstacleRect(MARGIN, MARGIN, MARGIN, 3000-MARGIN)
        obstacles["wallSouth"] = ObstacleRect(2000-MARGIN, 2000-MARGIN, MARGIN, 3000-MARGIN)
        obstacles["wallEast"] = ObstacleRect(MARGIN, 2000-MARGIN, MARGIN, MARGIN)
        obstacles["wallWest"] = ObstacleRect(MARGIN, 2000-MARGIN, 3000-MARGIN, 3000-MARGIN)

        # Bases
        baseHome = ObstacleRect(0, 450+MARGIN, 2550-MARGIN, 3000)
        baseAway = ObstacleRect(0, 450+MARGIN, 0, 450+MARGIN)
        if color == HOME:
            obstacles["oppBase"] = baseAway
        else:
            obstacles["oppBase"] = baseHome

        # Plants
        for i, plant in enumerate(PLANTS_POS):
            obstacles[f"plant{i}"] = ObstacleCirc(*plant, radius=75+MARGIN)

        # Pots
        for i, pot in enumerate(POTS_POS):
            obstacles[f"pot{i}"] = ObstacleCirc(*pot[:2], radius=75+MARGIN)

    log_info("Number of static obstacles : {}.".format(len(obstacles)))
   
    return obstacles
