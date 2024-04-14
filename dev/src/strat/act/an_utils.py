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

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import rospy
from math import pi
from an_const import NODE_NAME, COLOR

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

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
    Print fatal errors (programs cannot continue to run)
    """
    rospy.logfatal(f"{NODE_NAME} {msg}")
    

def adapt_pos_to_side(x, y, theta, color):
    if color == 0:
        return x, y, theta
    else:
        return x, 3000-y, -theta        # si la symétrie est selon l'axe y
        # return 2000-x, y, theta + pi  # si la symétrie est selon l'axe x
     

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

#################################################################
# Debug functions												#
#################################################################

# Enable or disable debug prints
debug_prints = True  

# Debug print function
def debug_print(format, *msgs):
    
    # If debug prints are disabled, quit
    if not debug_prints: return

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