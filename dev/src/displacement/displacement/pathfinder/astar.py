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
@file: astar.py
@status: in progress

Librairie implementant un algorithme de A* sur un ensemble de noeud et 
d'obstacle définie dans la classe Map.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import time
import numpy as np
import pyastar2d
import math

from disp_utils import GRID_INTERVAL, log_info
from pathfinder.exceptions import PathNotFoundError, TimeOutError, DestBlockedError

### CONSTANTES ########################################################

#######################################################################

def can_go_straight(tableMap, init, goal):
    # Can't use init == goal because this does not support np arrays
    # Needs to support both list and np arrays
    if init[0] == goal[0] and init[1] == goal[1]:
        return False

    segment = np.zeros((2,2))
    segment[0] = init
    segment[1] = goal

    obstacles = list(tableMap.get_obstacles())
    for obstacle in obstacles:
        if obstacle.crosses(segment):
            return False
    return True

#######################################################################
#
#                           Algo A star
#
#######################################################################

def a_star(init, goal, tableMap, weights, _maxAstarTime):

    start_time = time.perf_counter()

    if not tableMap.get_avoid():
        return [goal]

    if can_go_straight(tableMap, init, goal[:2]):
        return [goal]

    final_cap = goal[2]
   
    weights[:] = 1 # avoids recreating the array every time
    if tableMap.get_avoid():
        obstacles = list(tableMap.get_obstacles()) # Avoid concurrent list change during iteration
        for obstacle in obstacles:
            bb = obstacle.bounding_box() / GRID_INTERVAL
            for i in range(math.floor(bb[0,0]), min(math.floor(bb[1,0]+1), weights.shape[0])):
                for j in range(math.floor(bb[0,1]), min(math.floor(bb[1,1]+1), weights.shape[1])):
                    if weights[i,j] == 1 and obstacle.is_node_in(i*GRID_INTERVAL, j*GRID_INTERVAL):
                        weights[i,j] = None
                        
    start = [min(int(round(init[0] / GRID_INTERVAL, 0)), weights.shape[0]-1), min(int(round(init[1] / GRID_INTERVAL, 0)), weights.shape[1]-1)]
    dest = [min(int(round(goal[0] / GRID_INTERVAL, 0)), weights.shape[0]-1), min(int(round(goal[1] / GRID_INTERVAL, 0)), weights.shape[1]-1)]

    pre_process_time = time.perf_counter()
   
    path = pyastar2d.astar_path(weights, start, dest, allow_diagonal=True)
    if path is None:
        raise PathNotFoundError
        
    astar_time = time.perf_counter()

    path *= GRID_INTERVAL
    path[-1] = goal[:2]
    path = list(path)

    # POST-PROCESSING
    # pyastar2d only allows 8-connexity, but this might not be optimal
            
    def reduce_path(tableMap, path):
        if len(path) > 2:
            if can_go_straight(tableMap, path[0], path[-1]):
                return [path[0], path[-1]]
            else:
                i = 0 # first connectable to first node
                j = len(path) # first non-connectable to first node
                while j - i > 1:
                    h = (i + j) // 2
                    if can_go_straight(tableMap, path[0], path[h]):
                        i = h
                    else:
                        j = h

                if i == 0:
                    return [path[0]] + reduce_path(tableMap, path[i+1:])
                else:
                    return [path[0]] + reduce_path(tableMap, path[i:])

        return path

    path = reduce_path(tableMap, path)

    path_with_caps = []
    for i in range(len(path)):
        if i < len(path) - 1:
            seg = path[i+1] - path[i]
            cap = np.arctan2(seg[1], seg[0])
            path_with_caps.append([*path[i], cap.item()])
        else:
            path_with_caps.append([*path[i], final_cap])

    post_process_time = time.perf_counter()

    log_info(f"Pre-process duration: {pre_process_time - start_time}s")
    log_info(f"Astar duration: {astar_time - pre_process_time}s")
    log_info(f"Post-process duration: {post_process_time - astar_time}s")

    return path_with_caps[1:]

