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
@file: obstacle_circ.py
@status: OK.
"""

import math

class ObstacleCirc:
    
    """Implemente un obstacle circulaire sur la map."""

    def __init__(self, xCenter, yCenter, radius):
        """Initialization of obstacle."""
        self.name = "C"         # Type d'obstacle
        self.xCenter = xCenter  # Coordonnée selon l'axe X du centre du cercle
        self.yCenter = yCenter  # Coordonnée selon l'axe Y du centre du cercle
        self.radius = radius    # Rayon du cercle
        
    def getXCenter(self):
        return self.xCenter
    
    def getYCenter(self):
        return self.yCenter
    
    def getRadius(self):
        return self.radius
    
    def getName(self):
        return self.name

    def isNodeIn(self, node):
        """Verifie si le node passe en param est dans l'obstacle."""
        x = node.getX()
        y = node.getY()
        return math.sqrt((x-self.xCenter)**2+(y-self.yCenter)**2)<self.radius