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



READER = configparser.ConfigParser()
try :
	READER.read(os.path.join(os.path.dirname(__file__),"../../gr_config.ini"))
except:
	print("no file found...")

ROBOT_NAME = READER.get("ROBOT", "robot_name")
INIT_POS = READER.get("ROBOT", "init_pos")

MAX_ASTAR_TIME = READER.get("PATHFINDER", "max_astar_time")

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

def to_robot_coord(x_robot, y_robot, cap, pos):
    """Fonction transposant pos dans le repere local du robot."""
    x_loc = np.cos(cap)*(pos[0]-x_robot)+np.sin(cap)*(pos[1]-y_robot)
    y_loc = np.cos(cap)*(pos[1]-y_robot)-np.sin(cap)*(pos[0]-x_robot)
    return (x_loc, y_loc)

def printable_pos(pos):
    """Fonction de mise en forme d'une position pour affichage."""
    p_pos = [int(pos[0]), int(pos[1]), round(pos[2], 2)]
    return p_pos

def patch_frame_br(x, y, theta, color):
    """Easier patch"""
    if COLOR[color] == "HOME":
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

