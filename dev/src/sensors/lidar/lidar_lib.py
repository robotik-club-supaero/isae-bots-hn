#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file: lidar_lib.py
@status: OK

Fichier librarie contenant les fonctions utiles au traitement des
donnees des obstacles detectes par le LIDAR.
"""

#######################################################################
# IMPORTS
#######################################################################

from __future__ import division

import math
import numpy as np

#######################################################################
# CONSTANTS
#######################################################################

TABLE_MARGIN = 100      # marge aux bords de table
TABLE_H = 3000          # hauteur de table
TABLE_W = 2000          # largeur de table

LOCAL_LIM = 100         # distance lim de regroupement/localisation

#######################################################################
# FUNCTIONS
#######################################################################

#calcule les coordonnées absolues d'un set de points du lidar (on applique déjà un masque pour supprimer les données en dehors de la table ou dans la balance)
def absol_coord(x_r,y_r,cap,ranges,angle_min,angle_max,angle_inc,range_min,range_max):
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

    if obstList != []:
        # S'il y a des obstacles
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

    else:
        # Pas d'obstacles. 
        return []
        

def need_stop():
    pass


