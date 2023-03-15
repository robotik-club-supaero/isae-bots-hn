#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
if os.environ['USER'] == 'pi':
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg 		# sur robot
else:
    from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################

class Maps:

    """ Classe représentant le terrain de jeu """

#######################################################################
# Methods
#######################################################################

    def __init__(self, standardNodeList, avoidingNodeList, obstacleList):
        """Initialization of Maps."""
        self.robotWidth = int(literal_eval(READER.get("Robot", "robot_larg")))

        self.obstacleList = obstacleList            # Liste des obstacles
        self.standardNodeList = standardNodeList    # Liste des noeuds de passages présents sur la Map        
        self.avoidingNodeList = avoidingNodeList    # Liste des noeud à utiliser lors de l'évitement

        self.obstacleRobotPos = None                # Position robot à éviter

        # Choix map classique ou map d'évitement    #### ATRANSFORMER EN ENTIER POUR AVOIR PLUS QUE 2 MAPS ####
        self.avoid = False
        # Deuxieme essai evitement
        self.isSecondAttempt = False

    def getObstacleList(self):
        if self.avoid:
            if self.isSecondAttempt:
                self.obstacleList.pop()
            return self.obstacleList+[self.obstacleRobotPos]
        else:
            return self.obstacleList
    
    def getNodeList(self):
        if self.avoid:
            return self.avoidingNodeList
        else:
            return self.standardNodeList

    def setObstacleRobotPos(self, obstacleRobotPos):
        self.obstacleRobotPos = obstacleRobotPos

    def setAvoid(self, avoid, isSecondAttempt):
        self.avoid = avoid
        self.isSecondAttempt = isSecondAttempt

    def getAvoid(self):
        return self.avoid