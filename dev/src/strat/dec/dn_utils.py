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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import rospy
import configparser
from enum import IntEnum, Enum

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

NODE_NAME = "[DEC]"
# SIMULATION = False if os.environ['HOSTNAME'] in ['pr', 'gr'] else True

#################################################################
# CONFIG 
READER = configparser.ConfigParser()
READER.read(os.path.join(os.path.dirname(__file__),'../../robot_config.cfg'))

#################################################################
# WINDOW
TERM_SIZE = 62

#################################################################
# ROBOTS PARAMS
class ROBOT_SIDES(IntEnum):
    HOME = 0
    AWAY = 1

#################################################################

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}


#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
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
        
def log_info(msg):
    """
    Print standard logs.
    """
    rospy.loginfo(f"{NODE_NAME} {msg}")


def log_warn(msg):
    """
    Print warning logs.
    """
    rospy.logwarn(f"{NODE_NAME} {msg}")

def log_errs(msg):
    """
    Print errors logs.
    """
    rospy.logerr(f"{NODE_NAME} {msg}")

def log_fatal(msg):
    """
    Print errors logs.
    """
    rospy.logfatal(f"{NODE_NAME} {msg}")