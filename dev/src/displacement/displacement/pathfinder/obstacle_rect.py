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

import numpy as np

class ObstacleRect:
    
    """Implemente un obstacle rectangulaire sur la map."""

    def __init__(self,x_min,x_max,y_min,y_max):
        """Initialization of obstacle."""    
        self.name = "R"     # Type d'obstacle
        self.coords = np.array([[x_min, y_min], [x_max, y_max]])
    
    def __str__(self):
        return f"ObstacleRect(x_min={self.x_min}, x_max={self.x_max}, y_min={self.y_min}, y_max={self.y_max})"
         
    @property
    def x_min(self):
        return self.coords[0,0]
    
    @property
    def x_max(self):
        return self.coords[1,0]
    
    @property
    def y_min(self):
        return self.coords[0,1]
    
    @property
    def y_max(self):
        return self.coords[1,1]
    
    def get_name(self):
        return self.name

    def is_node_in(self, x, y):
        return self.x_min<=x<=self.x_max and self.y_min<=y<=self.y_max

    def crosses(self, segment):  
        x_min = self.x_min
        x_max = self.x_max
        y_min = self.y_min
        y_max = self.y_max

        origin_x = segment[0,0]
        origin_y = segment[0,1]

        line_vect = segment[1] - segment[0]
        if np.linalg.norm(line_vect) != 0:
            line_vect /= np.linalg.norm(line_vect)
        node_dist = np.linalg.norm(line_vect)

        if line_vect[0] != 0 :
            t_min = (x_min-origin_x)/line_vect[0]
            t_max = (x_max-origin_x)/line_vect[0]

            if (y_min<origin_y+t_min*line_vect[1]<y_max and 0<t_min<node_dist) or (y_min<origin_y+t_max*line_vect[1]<y_max and 0<t_max<node_dist):

                return True

        if line_vect[1] != 0 :
            t_min = (y_min-origin_y)/line_vect[1]
            t_max = (y_max-origin_y)/line_vect[1]
            
            if (x_min<origin_x+t_min*line_vect[0]<x_max and 0<t_min<node_dist ) or (x_min<origin_x+t_max*line_vect[0]<x_max and 0<t_max<node_dist):

                return True
                
        return False

    def bounding_box(self):
        return self.coords
    
    def bb_corners(self, x, y):

        pos = np.array([x, y])
        shape = [[self.x_min, self.y_min], [self.x_max, self.y_min], [self.x_min, self.y_max], [self.x_max, self.y_max]]

        corners = []

        for corner in shape:
            segment = np.zeros((2,2))
            segment[0] = pos
            segment[1] = np.array(corner)

            if self.crosses(segment) or np.array_equal(pos, np.array(corner)):
                continue
            corners.append(corner)

        print(corners)
        return corners