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
@file: astar_redefined.py
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
import heapq

from ..disp_utils import GRID_INTERVAL
from .exceptions import PathNotFoundError, TimeOutError, DestBlockedError

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

def a_star(init, goal, tableMap, weights, _maxAstarTime, logger):

    print(_maxAstarTime)

    start_time = time.perf_counter()

    if not tableMap.get_avoid():
        print("End of a_star if in no avoid mode")
        return [goal]

    if can_go_straight(tableMap, init, goal[:2]):
        print("End of a_star if can go straight")
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
   
    # path = pyastar2d.astar_path(weights, start, dest, allow_diagonal=True)
    # Grid reducer: idea --> remove from grid the positions corresponding to empty zones and obstacles, only keep borders of obstacles

    path = astar_path(tableMap, start, dest, _maxAstarTime)

    print("end of a_star algorithm")

    if path is None:
        raise PathNotFoundError
        
    astar_time = time.perf_counter()

    path *= GRID_INTERVAL
    path[-1] = goal[:2]
    path = list(path)

    # POST-PROCESSING
    # pyastar2d only allows 8-connexity, but this might not be optimal
        
    # path = reduce_path(tableMap, path)

    path_with_caps = []
    for i in range(len(path)):
        if i < len(path) - 1:
            seg = np.array(path[i+1]) - np.array(path[i])
            cap = np.arctan2(seg[1], seg[0])
            path_with_caps.append([*np.array(path[i]), cap.item()])
        else:
            path_with_caps.append([*np.array(path[i]), final_cap])

    post_process_time = time.perf_counter()

    logger.debug(f"Pre-process duration: {pre_process_time - start_time}s")
    logger.debug(f"Astar duration: {astar_time - pre_process_time}s")
    logger.debug(f"Post-process duration: {post_process_time - astar_time}s")

    print("End of a_star elsewise")
    return path_with_caps[1:]


#######################################################################
#
#                       Test Algo A star no pyastar
#
#######################################################################

def astar_path(tableMap, start, goal, strMaxTime):

    init_time = time.perf_counter()

    visited = []
    openQ = []
    start_state = (0, start, [], 0) 
    # le state contient dans l'ordre (total, position, path, coût)
    # heapq trie les éléments en fonction de la première variable, donc total
    heapq.heappush(openQ, start_state)

    while len(openQ) >0:
        if time.perf_counter() - init_time >= float(strMaxTime):
            print("No path was found in time")
            return None

        s = heapq.heappop(openQ)
        visited.append(s[1])

        if s[1][0] == goal[0] and s[1][1] == goal[1]:
            # on a trouvé le path otpimal vers l'objectif
            print("showing path")
            print(s[2])
            print("end showing path")

            return s[2]

        if can_go_straight(tableMap, s[1], goal):
            # on voit l'objectif, et on l'ajoute aux points explorables, on ne le sort pas encore
            g = cost(s[1], goal) + s[3]
            path = s[2].copy()
            # path.append(np.array(goal))
            path.append(goal)
            ns = (g, goal, path, g)
            heapq.heappush(openQ, ns)

        obstacles = tableMap.get_obstacles()

        for obstacle in obstacles:
            corners = obstacle.bb_corners(s[1][0], s[1][1])
            for corner in corners:
                # print(corner in visited)
                if corner in visited:
                    continue
                else:
                    if can_go_straight(tableMap, s[1], corner):
                        c = cost(s[1], corner) + s[3] # calcul du nouveau coût
                        g = c + heuristic(corner, goal) # calcul de la fontcion obj totale (coût + heuristic)
                        path = s[2].copy()
                        # path.append(np.array(corner))
                        path.append(corner)
                        ns = (g, corner, path, c)
                        # print(ns)
                        heapq.heappush(openQ, ns)

    return None


def cost(pos, obj):

    dist = np.sqrt((pos[0]-obj[0])**2 + (pos[1]-obj[1])**2)
    return dist

def heuristic(pos, goal):
    '''
    Implémentation de l'heuristique àl'aide de la norme 2, qui domine la
    norme infinie (ie meilleur indicateur pour le pathfinding), et reste
    admissible (ie: ne surestime pas la distance réelle)
    '''

    dist = np.sqrt((pos[0]-goal[0])**2 + (pos[1]-goal[1])**2)
    return dist






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