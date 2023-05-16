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
from pathfinder.obstacle_rect import ObstacleRect
from pathfinder.obstacle_circ import ObstacleCirc
from pathfinder.obstacle_tria import ObstacleTria

from disp_utils import *

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

    # On rentre les obstacles avec cotes precises et on y rajoute la 
    # largeur du robot qu'on simule
    robotW = int(READER.get("ROBOT", "robot_larg"))
    robotL = int(READER.get("ROBOT", "robot_long"))
    robotDiag = np.linalg.norm([robotW/2, robotL/2])
    margin = robotDiag // 2 + 20

    # Walls 
    wallNorth = ObstacleRect(0, 0, 0, 3000)
    wallSouth = ObstacleRect(2000, 2000, 510, 2490)
    wallEast = ObstacleRect(0, 2000, 0, 0)
    wallWest = ObstacleRect(0, 2000, 3000, 3000)

    # Bases
    baseHome = ObstacleRect(400-margin, 1000+margin, 0, 400+margin)
    baseAway = ObstacleRect(400-margin, 1000+margin, 2600-margin, 3000)

    # Cherries
    cherriesPerpendicular1 = ObstacleRect(985, 1015, 0, 300)
    cherriesPerpendicular2 = ObstacleRect(985, 1015, 2700, 3000)
    cherriesWall1 = ObstacleRect(0, 30, 1350, 1650)
    cherriesWall2 = ObstacleRect(1970, 2000, 1350, 1650)

    # Samples
    # s_radius = 150 / 2
    # s_margin = 10

    # samplesCoor = [(555, 900), (675, 830), (795, 900)]
    samplesList = []
    # for x,y in samplesCoor:
    #     samplesList.append( ObstacleCirc(x,y,s_radius+s_margin+margin))

    obstacleList = samplesList

    if color == HOME:
        obstacleList.extend([wallNorth, wallEast, wallWest, wallSouth])
        obstacleList.extend([cherriesPerpendicular1, cherriesPerpendicular2, cherriesWall1, cherriesWall2])

        #obstacleList.append(baseHome)
    else:
        obstacleList.extend([wallNorth, wallEast, wallWest, wallSouth])
        obstacleList.extend([cherriesPerpendicular1, cherriesPerpendicular2, cherriesWall1, cherriesWall2])

        #obstacleList.append(baseAway)

    log_info("Number of static obstacles : {}.".format(len(obstacleList)))
    return obstacleList
