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
import numpy as np

class ObstacleCirc:
    
    """Implemente un obstacle circulaire sur la map."""

    def __init__(self, x_center, y_center, radius):
        """Initialization of obstacle."""
        self.name = "C"         # Type d'obstacle
        self.center = np.array([x_center, y_center])
        self.radius = radius    # Rayon du cercle

    def __str__(self):
        return f"ObstacleCirc(x={self.get_x_center()}, y={self.get_y_center()}, radius={self.radius})"
        
    def get_x_center(self):
        return self.center[0]
    
    def get_y_center(self):
        return self.center[1]
    
    def set_x_center(self, x):
        self.center[0] = x

    def set_y_center(self, y):
        self.center[1] = y

    def get_radius(self):
        return self.radius
    
    def get_name(self):
        return self.name

    def is_node_in(self, x, y):
        """Verifie si le node passe en param est dans l'obstacle."""
        pos = np.array([x, y])
        return np.linalg.norm(self.center-pos) <= self.radius

    def crosses(self, segment):      
        center = self.center

        OA = segment[0] - center
        OB = segment[1] - center
        if np.dot(OA, segment[0] - segment[1]) > 0 and np.dot(OB, segment[1] - segment[0]) > 0:
            prod = np.abs(OA[0] * OB[1] - OA[1] * OB[0])
            dist = prod / np.linalg.norm(segment[1] - segment[0])
            return dist < self.radius
        else:
            return np.minimum(np.linalg.norm(OA), np.linalg.norm(OB)) < self.radius

    def bounding_box(self):
        return np.array([self.center - self.radius, self.center + self.radius])
    
    def bb_corners(self, x, y):

        pos = np.array([x, y])

        direct = self.center-pos
        rad = self.radius
        dist = np.linalg.norm(direct)

        if rad >= dist:
            return []

        tan_norm = np.sqrt(dist**2 - rad**2)

        cos_tan = tan_norm/dist
        sin_tan = rad/dist
        vec_tan_pos = np.dot(direct, np.array([[cos_tan, sin_tan], [-sin_tan, cos_tan]]))
        vec_tan_neg = np.dot(direct, np.array([[cos_tan, -sin_tan], [sin_tan, cos_tan]]))
        
        face = (dist-rad) * direct / dist
        face = face.tolist()
        tan_pos = (pos + vec_tan_pos).tolist()
        tan_neg = (pos + vec_tan_neg).tolist()
        corners = [tan_pos, tan_neg, face]

        return corners
    
    def corners(self):
        radx = 0.9*np.array([self.radius,0])
        rady = 0.9*np.array([0,self.radius])
        cent = np.array(self.center)

        corners = []
        corners.append((cent+radx+rady).tolist())
        corners.append((cent+radx-rady).tolist())
        corners.append((cent-radx+rady).tolist())
        corners.append((cent-radx-rady).tolist())

        return corners