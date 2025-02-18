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
# import pyastar2d
import math
import heapq

from ..disp_utils import GRID_INTERVAL
from .exceptions import PathNotFoundError, TimeOutError, DestBlockedError


#######################################################################
#
#                    CLASS USED FOR THE PATHFINDING
#
#######################################################################

class Point_astar:
    def __init__(self, x: float, y: float):
        self.coords = (x, y)
        self.children = []
        self.heuristic = -1
    
    def isDifferent(self, point):
        return not(self.coords[0] == point.getCoords()[0] and self.coords[1] == point.getCoords()[1])
    
    def getCoords(self):
        return self.coords
    
    def getChildren(self):
        return self.children

    def updateChildren(self, map, tableMap):
        self.eraseChildren()
        for point in map.getPoints():
            if can_go_straight(tableMap, self.coords, point.getCoords()) and self.isDifferent(point):
                self.children.append(Segment_astar(self, point))
    
    def eraseChildren(self):
        self.children = []

    def setHeuristic(self, map):
        self.heuristic = map.heuristic(self)

    def getHeuristic(self):
        return self.heuristic

    def addMapData(self, map, tableMap):
        self.updateChildren(map, tableMap)
        self.setHeuristic(map)

    def rmvMapData(self):
        self.eraseChildren()
        self.heuristic = -1

    def __lt__(self, other):
        """Less than comparator"""
        return self.coords < other.coords
    
    def __eq__(self, other):
        """Equality comparator"""
        return self.coords == other.coords
    
    def __gt__(self, other):
        """Greater than comparator"""
        return self.coords > other.coords
    
    def __le__(self, other):
        """Less than or equal comparator"""
        return self.coords <= other.coords
    
    def __ge__(self, other):
        """Greater than or equal comparator"""
        return self.coords >= other.coords


class Segment_astar:
    def __init__(self, init: Point_astar, dest: Point_astar):
        self.init   = init
        self.dest   = dest
        self.length = self.calculateLength()

    def getInit(self):
        return self.init
    
    def getDest(self):
        return self.dest
    
    def getLength(self):
        return self.length

    def calculateLength(self):
        pos = self.init.getCoords()
        obj = self.dest.getCoords()
        return np.sqrt((pos[0]-obj[0])**2 + (pos[1]-obj[1])**2)
    

class Map_astar:
    def __init__(self, init: Point_astar, dest: Point_astar):
        self.points = [init, dest]
        self.goal  = dest
        self.start  = init

    def addPoint(self, point: Point_astar):
        self.points.append(point)

    def getPoints(self):
        return self.points
    
    def getStart(self):
        return self.start
    
    def getGoal(self):
        return self.goal
    
    def heuristic(self, point: Point_astar):
        goal = self.goal.coords
        pos  = point.coords
        return np.sqrt((pos[0]-goal[0])**2 + (pos[1]-goal[1])**2)
    
    def updateMap(self, tableMap):
        for point in self.points:
            Point_astar.addMapData(self=point, map=self, tableMap=tableMap)




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

    start_time = time.perf_counter()

    if not tableMap.get_avoid():
        #print("End of a_star if in no avoid mode \n")
        return [goal]

    """
    if can_go_straight(tableMap, init, goal[:2]):
        #print("End of a_star if can go straight \n")
        return [goal]
    """
        
    final_cap = goal[2]

    # start = [min(int(round(init[0] / GRID_INTERVAL, 0)), weights.shape[0]-1), min(int(round(init[1] / GRID_INTERVAL, 0)), weights.shape[1]-1)]
    # dest = [min(int(round(goal[0] / GRID_INTERVAL, 0)), weights.shape[0]-1), min(int(round(goal[1] / GRID_INTERVAL, 0)), weights.shape[1]-1)]

    start = [min(int(init[0]), GRID_INTERVAL*(weights.shape[0]-1)), min(int(init[1]), GRID_INTERVAL*(weights.shape[1]-1))]
    dest = [min(int(goal[0]), GRID_INTERVAL*(weights.shape[0]-1)), min(int(goal[1]), GRID_INTERVAL*(weights.shape[1]-1))]


    astarMap = Map_astar(Point_astar(start[0], start[1]), Point_astar(dest[0], dest[1]))
    
    obstacles = tableMap.get_obstacles()

    # on crée la map avant d'exécuter l'algo
    for obstacle in obstacles:
        corners = obstacle.corners()
        for corner in corners:
            astarMap.addPoint(Point_astar(corner[0], corner[1]))
    
    astarMap.updateMap(tableMap)  # Toujours bien penser a update la map après construction pour ajouter les connexions  


    pre_process_time = time.perf_counter()


    #path = astar_path(tableMap, astarMap, start, dest, _maxAstarTime) # see implementation at the end --> objective is to implement such method in C++
    path = astar_path_v2(tableMap, astarMap, _maxAstarTime)

    # print("end of a_star algorithm \n")
    
    if path == -1:
        raise TimeOutError

    if path is None:
        raise PathNotFoundError
        
    astar_time = time.perf_counter()

    path = np.array(path)
    path[-1] = goal[:2]
    path = list(path)

    # POST-PROCESSING

    path_with_caps = []
    for i in range(len(path)):
        if i < len(path) - 1:
            seg = np.array(path[i+1]) - np.array(path[i])
            cap = np.arctan2(seg[1], seg[0])
            path_with_caps.append([*path[i], cap.item()])
        else:
            path_with_caps.append([*path[i], final_cap])

    post_process_time = time.perf_counter()

    logger.debug(f"Pre-process duration: {pre_process_time - start_time}s")
    logger.debug(f"Astar duration: {astar_time - pre_process_time}s")
    logger.debug(f"Post-process duration: {post_process_time - astar_time}s")

    #print("End of a_star elsewise \n")
    return path_with_caps[1:]


#######################################################################
#
#                       Test Algo A star no pyastar
#
#######################################################################

def astar_path_v2(tableMap, map: Map_astar, strMaxTime):
    init_time = time.perf_counter()

    start = map.getStart()
    goal  = map.getGoal()

    visited = []
    openQ = []
    start_state = (0, start, [start.getCoords()], 0)
    # le state contient dans l'ordre (total, Point_astar, path = [(float, float)], coût)
    # heapq trie les éléments en fonction de la première variable, donc total, ainsi la priorité est donné au path avec le coût total le plus faible
    heapq.heappush(openQ, start_state)

    while len(openQ) >0:
        if time.perf_counter() - init_time >= float(strMaxTime):
            #print("No path was found in time \n")
            return -1

        s = heapq.heappop(openQ)
        pos = s[1]
        visited.append(pos.getCoords())

        if not(pos.isDifferent(goal)):
            path = s[2].copy()
            return path
        
        for vectNext in pos.getChildren():
            nextPos = vectNext.getDest()
            if nextPos.getCoords() in visited:
                continue
            else:
                cost = vectNext.getLength() + s[3]
                g    = cost + nextPos.getHeuristic()

                tpath = s[2].copy()
                tpath.append(nextPos.getCoords())

                nextS = (g, nextPos, tpath, cost)
                heapq.heappush(openQ, nextS)
