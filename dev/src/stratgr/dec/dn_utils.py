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

_NODENAME_ = "DEC"
#SIMULATION = False if os.environ['USER'] == 'pi' else True

#################################################################
# CONFIG 
CONFIG_READER = configparser.ConfigParser()
CONFIG_READER.read(os.path.join(os.path.dirname(__file__),'../../../gr_config.cfg'))

#################################################################
# WINDOW
TERM_SIZE = 62

#################################################################
# ROBOTS PARAMS
class ROBOT_SIDES(IntEnum):
    HOME = 0
    AWAY = 1

#################################################################
# STRATS PARAMS
class STRAT_INDEX(IntEnum):
    HOMOLOGATION = 0
    TESTS = 1
    MATCH = 2

class STRAT_NAMES(str, Enum):
    HOMOLOGATION = "homologation_strat"
    TESTS = "tests_strat"
    MATCH = "match_strat"

class DN_LIST_ACTION_INDEX(IntEnum):
    PARK = 0
    END = 1
    PREEMPTED = 2
    #...
    #...

class DN_LIST_ACTION_NAMES(str, Enum):
    PARK = 'park'
    END = 'end'
    PREEMPTED = 'preempted'
    #... 

class CB_NEXT_ACTION(IntEnum):
    NONE    = -3
    PARK_IT = -2  # park interrupt
    STOP_IT = -1  # stop interrupt
    #ACTION_1 = 0
    #ACTION_2 = 0
    #ACTION_3 = 0
    #ACTION_4 = 0
    #ACTION_5 = 0
    #ACTION_6 = 0
    #ACTION_7 = 0
    #...



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

def log_info(log):
    """
    Print standard logs.
    """
    rospy.loginfo(f"{Color.WHITE}[{_NODENAME_}] {log}{Color.RESET}")


def log_warn(log):
    """
    Print warning logs.
    """
    rospy.logwarn(f"{Color.YELLOW}[{_NODENAME_}] {log}{Color.RESET}")


def log_errs(log):
    """
    Print errors logs (errors concerning actions, not critical)
    """
    rospy.logerr(f"{Color.RED}[{_NODENAME_}] {log}{Color.RESET}")


def log_fatal(log):
    """
    Print fatal error logs (critical errors, not concerning actions and never supposed to happen)
    """
    rospy.logfatal(f"{Color.BOLD}{Color.UNDERLINE}{Color.RED}[{_NODENAME_}] {log}{Color.RESET}")