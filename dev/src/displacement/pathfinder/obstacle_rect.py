#!/usr/bin/env python
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
@file: obstacle_rect.py
@status: OK.
"""

class ObstacleRect:
    
    """Implemente un obstacle rectangulaire sur la map."""

    def __init__(self,xMin,xMax,yMin,yMax):
        """Initialization of obstacle."""    
        self.name = "R"     # Type d'obstacle
        self.xMin = xMin    # xMin du rectangle
        self.xMax = xMax    # xMax du rectangle
        self.yMin = yMin    # yMin du rectangle
        self.yMax = yMax    # yMax du rectangle
        
    def getXMin(self):
        return self.xMin
    
    def getXMax(self):
        return self.xMax
    
    def getYMin(self):
        return self.yMin
    
    def getYMax(self):
        return self.yMax
    
    def getName(self):
        return self.name

    def isNodeIn(self, node):
        x = node.getX()
        y = node.getY()
        return self.xMin<x<self.xMax and self.yMin<y<self.yMax

    