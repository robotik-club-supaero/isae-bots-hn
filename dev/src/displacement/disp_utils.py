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
from pathfinder.obstacle_circ import ObstacleCirc

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

DEBUG_PRINT = True

VERSION = 1
NODE_NAME = "[DSP]"   



READER = configparser.ConfigParser()
try :
	READER.read(os.path.join(os.path.dirname(__file__),"../robot_config.cfg"))
except:
	print("no file found...")

ROBOT_NAME = READER.get("ROBOT", "robot_name")
INIT_POS = list(literal_eval(READER.get("ROBOT", "init_pos")))
INIT_POS2 = list(literal_eval(READER.get("ROBOT", "init_pos2")))
INIT_POS3 = list(literal_eval(READER.get("ROBOT", "init_pos3")))
INIT_ZONE = int(READER.get("STRAT", "init_zone"))

MAX_ASTAR_TIME = READER.get("PATHFINDER", "max_astar_time")

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

robotW = int(READER.get("ROBOT", "robot_larg"))
robotL = int(READER.get("ROBOT", "robot_long"))
robotDiag = np.linalg.norm([robotW/2, robotL/2])
MARGIN = robotDiag // 2 + 20


ONE_PI = np.pi
HLF_PI = np.pi/2
QRT_PI = np.pi/4

def to_robot_coord(x_robot, y_robot, cap, pos):
    """Fonction transposant pos dans le repere local du robot."""
    x_loc = np.cos(cap)*(pos[0]-x_robot)+np.sin(cap)*(pos[1]-y_robot)
    y_loc = np.cos(cap)*(pos[1]-y_robot)-np.sin(cap)*(pos[0]-x_robot)
    return (x_loc, y_loc)

def printable_pos(pos):
    """Fonction de mise en forme d'une position pour affichage."""
    p_pos = [int(pos[0]), int(pos[1]), round(pos[2], 2)]
    return p_pos

# def patch_frame_br(x, y, theta, color):
#     """Easier patch"""
#     if COLOR[color] == "HOME":
#         return x, y, theta
#     return 2000-x, y, theta+ONE_PI

#######################################################################
# LOGS functions
#######################################################################

def log_info(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.loginfo(f"{NODE_NAME} {msg}")

def log_errs(msg):
    """Fonction intermediaire affichant les logs d'erreurs."""
    rospy.logerr(f"{NODE_NAME} {msg}")

def log_warn(msg):
    """Fonction intermediaire affichant les logs de warning."""
    rospy.logwarn(f"{NODE_NAME} {msg}")



#################################################################
# Colors gestion												#
#################################################################

class Color():
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'

color_dict = {'n':Color.BLACK, 'r':Color.RED, 'g':Color.GREEN, 'y':Color.YELLOW, 'b':Color.BLUE, 
             'm':Color.MAGENTA, 'c':Color.CYAN, 'w':Color.WHITE}



# Debug print function
def debug_print(format, *msgs):
    
    # If debug prints are disabled, quit
    if not DEBUG_PRINT: return

    # If no color was specified, error & quit
    if len(format) == 0:
        print(Color.RED + "Wrong debug_print color" + Color.RESET)
        return

    print_string = ""
    color = format[0]

    if len(format[1:]) > 0:
        shape = format[1:]
        if shape == '*': 
            print_string += Color.BOLD
        elif shape == '-': 
            print_string += Color.UNDERLINE
        elif shape == '*-': 
            print_string += Color.BOLD + Color.UNDERLINE

    total_msg = ""
    for msg in msgs:
        total_msg = total_msg + str(msg) + ", "
    total_msg = total_msg[:-2]

    try:
        print_string += color_dict[color] + total_msg + Color.RESET
        log_info(print_string)
    except KeyError:
        log_info(Color.RED + "Wrong debugPrint color" + Color.RESET)
        return