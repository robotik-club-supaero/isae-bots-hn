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
@file: maps.py
@status: OK.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import os
import numpy as np
from ast import literal_eval

from disp_utils import READER, GRID_INTERVAL
#################################################################################################
""" if os.environ['USER'] == 'pi':
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg 		# sur robot
else: """
from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################
# from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

class Maps:

    """ Classe représentant le terrain de jeu """

#######################################################################
# Methods
#######################################################################

    def __init__(self, obstacle_list):
        """Initialization of Maps."""
        self.robot_width = int(literal_eval(READER.get("ROBOT", "robot_larg")))

        self.obstacle_list = obstacle_list            # Dict des obstacles

        x = np.linspace(0, 2000, 2000//GRID_INTERVAL)
        y = np.linspace(0, 3000, 3000//GRID_INTERVAL)
        self.grid = np.zeros((x.shape[0], y.shape[0], 2), dtype=np.float32)

        x, y = np.meshgrid(x, y, copy=False, indexing='ij')
        self.grid[...,0] = x
        self.grid[...,1] = y

        # Choix map classique ou map d'évitement    #### ATRANSFORMER EN ENTIER POUR AVOIR PLUS QUE 2 MAPS ####
        self.avoid = False
   
    def get_grid(self):
        return self.grid

    def get_obstacles(self):
        return self.obstacle_list.values()
        
    def remove_obstacle(self, obstacle_id):
        self.obstacle_list.pop(obstacle_id, None)

    def set_obstacle_robot_pos(self, obstacle_robot_pos):
        if obstacle_robot_pos is not None:
            self.obstacle_list["robot_pos"] = obstacle_robot_pos
        else:
            self.obstacle_list.pop("robot_pos", None)
    
    def get_obstacle_robot_pos(self):
        return self.obstacle_list.get("robot_pos", None)

    def set_avoid(self, avoid):
        self.avoid = avoid

    def get_avoid(self):
        return self.avoid