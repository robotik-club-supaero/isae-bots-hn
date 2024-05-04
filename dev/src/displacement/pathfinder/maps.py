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
from ast import literal_eval

from disp_utils import READER

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

    def __init__(self, standard_node_list, avoiding_node_list, obstacle_list):
        """Initialization of Maps."""
        self.robot_width = int(literal_eval(READER.get("ROBOT", "robot_larg")))

        self.obstacle_list = obstacle_list            # Dict des obstacles
        self.standard_node_list = standard_node_list    # Liste des noeuds de passages présents sur la Map        
        self.avoiding_node_list = avoiding_node_list    # Liste des noeud à utiliser lors de l'évitement

        # Choix map classique ou map d'évitement    #### ATRANSFORMER EN ENTIER POUR AVOIR PLUS QUE 2 MAPS ####
        self.avoid = False

    def get_obstacles(self):
        return self.obstacle_list.values()
        
    def remove_obstacle(self, obstacle_id):
        self.obstacle_list.pop(obstacle_id, None)
    
    def get_node_list(self):
        if self.avoid:
            return self.avoiding_node_list
        else:
            return self.standard_node_list

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