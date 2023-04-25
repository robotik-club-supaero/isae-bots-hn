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
@file: disp_utils.py
@status: clean R3

File containing the utilitary functions and constants used in the disp-
lacement node package.
"""

#################################################################
#																#
# 							IMPORTS 							#
#																#
#################################################################

import os
import rospy
import numpy as np

import configparser
from ast import literal_eval

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

DEBUG_PRINT = True

VERSION = 1
NODE_NAME = "[DSP] "   

SIMULATION = False if os.environ['USER'] == 'pi' else True

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

def to_robot_coord(x_robot, y_robot, cap, pos):
    """Fonction transposant pos dans le repere local du robot."""
    x_loc = np.cos(cap)*(pos[0]-x_robot)+np.sin(cap)*(pos[1]-y_robot)
    y_loc = np.cos(cap)*(pos[1]-y_robot)-np.sin(cap)*(pos[0]-x_robot)
    return (x_loc, y_loc)

def printable_pos(pos):
    """Fonction de mise en forme d'une position pour affichage."""
    p_pos = [int(pos[0]), int(pos[1]), round(pos[2], 2)]
    return p_pos

def patch_frame_br(x, y, theta):
    """Easier patch"""
    if COLOR_MATCH == "HOME":
        return x, y, theta
    return 2000-x, y, -theta

#######################################################################
# LOGS functions
#######################################################################

def log_info(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.loginfo(NODE_NAME+msg)

def log_errs(msg):
    """Fonction intermediaire affichant les logs d'erreurs."""
    rospy.logerr(NODE_NAME+msg)

def log_warn(msg):
    """Fonction intermediaire affichant les logs de warning."""
    rospy.logwarn(NODE_NAME+msg)


def dprint(msg):
    if DEBUG_PRINT:
        print(" > {}".format(msg))

