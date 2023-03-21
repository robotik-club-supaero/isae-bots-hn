#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file: lidar_lib.py
@status: OK

Librairie de fonctions permettant le traitement des données reçues par le LiDAR.
"""

#######################################################################
# IMPORTS
#######################################################################

from __future__ import division

import math
import numpy as np
import os, sys
import rospy
import configparser

#######################################################################
# CONSTANTS
#######################################################################

TABLE_MARGIN = 100      # marge aux bords de table
TABLE_H = 3000          # hauteur de table (selon y)
TABLE_W = 2000          # largeur de table (selon x)

LOCAL_LIM = 100         # distance lim de regroupement/localisation

SIMULATION = False if os.environ['HOSTNAME'] == 'pi' else True

READER = configparser.ConfigParser()
ROBOT_NAME = READER.get("Robot", "robot_name")
if not SIMULATION: 
    READER.read(os.path.join(os.path.dirname(__file__),"../../start.ini"))
elif ROBOT_NAME == "PR":
    READER.read(os.path.join(os.path.dirname(__file__),"../../pr_start.ini"))
elif ROBOT_NAME == "GR":
    READER.read(os.path.join(os.path.dirname(__file__),"../../gr_start.ini")) 

COLOR_MATCH = READER.get("Robot", "color") # Couleur du côté duquel on joue
CONFIG_MATCH = READER.get("Robot", "config") # Permet d'avoir plusieurs configurations (positions de départs (utile pour la coupe 2023))

#######################################################################
# FUNCTIONS
#######################################################################

def LOG_INFO(msg):
    rospy.loginfo("[LID] :" + msg)

def handler(rcv_sig, frame):
	"""Force the node to quit on SIGINT, avoid escalating to SIGTERM."""
	LOG_INFO("ISB Node forced to terminate...")
	rospy.signal_shutdown(rcv_sig)
	sys.exit()

#calcule les coordonnées absolues d'un set de points du lidar (on applique déjà un masque pour supprimer les données en dehors de la table ou dans la balance)
def absol_coord(x_r, y_r, cap, ranges, angle_min, angle_max, angle_inc, range_min, range_max):
    """Fonction de calcul des coord absolues d'un obstacle.
    
    Cette fonction calcule les coordonnees absolues d'un set de points 
    du lidar. (On applique deja un masque pour supp les donnees en dehors
    de la table)."""

    # Parametres de calcul
    margin = TABLE_MARGIN
    theta = angle_min
    
    # Liste des coord des obstacles sur la table
    obstList = [] 
    for dist in ranges:
        if range_min<dist<range_max:
            # Calcul des coords absolue sur la carte
            x_obs=x_r+dist*1000*np.cos(cap+theta)
            y_obs=y_r+dist*1000*np.sin(cap+theta)
            # On ne conserve que les obstacles sur la table..
            if (margin < y_obs < TABLE_H-margin) and (margin < x_obs < TABLE_W-margin):
                obstList.append([x_obs, y_obs])
        theta+=angle_inc
    return obstList
    

def dist(pt1, pt2):
    """Returns the distance between the 2 points. """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
 

#quand on a un set de points, cette fonction permet de déterminer un obstacle pour chaque regroupement de points   
def localisation(obstList):
    """Fonction qui permet de determiner un obstacle pour chaque groupe
    de points."""

    opponentList = []

    if obstList == []: # Si pas d'obstacles
        return []
    
    else :  # Si obstacles
        lastPos = obstList[0]
        xBary = 0
        yBary = 0
        nbPts = 0

        for pos in obstList:
            # On regarde si le point suivant fait partie du regourpement
            # precedent en regardant sa distance au regroupement
            if dist(lastPos,pos) < LOCAL_LIM:
                nbPts += 1
                xBary += pos[0]
                yBary += pos[1]
                lastPos = pos
                continue
            
            # Sinon, s'il y a plus d'un pt on moyennise
            if nbPts >= 1:
                xBary /= nbPts
                yBary /= nbPts 
                opponentList.append([xBary, yBary])
            # Et on cree un nouvel obstacle avec le point
            nbPts=1
            xBary = pos[0]
            yBary = pos[1]
            lastPos = pos
        
        if nbPts >= 1:
            xBary /= nbPts
            yBary /= nbPts 
            opponentList.append([xBary, yBary])
        return opponentList
        

def need_stop():
    pass


