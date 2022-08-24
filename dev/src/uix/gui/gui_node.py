#!/usr/bin/env python3
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

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import sys

import signal
import rospy
import time

from gui_msgs import init_msgs
from gui_utils import log_info, log_warn, log_errs

from mvc.Model import Model


def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT
    """
    print("Quit on SIGINT")

    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()



#################################################################
#                                                               #
#                           GUI node                            #
#                                                               #
#################################################################

class GuiNode:
    """
    GUI node: ROS node to display what happens with the robot
    """

    # Imported methods
    from gui_msgs import init_msgs, update_start, update_color

    def __init__(self):

        log_info("Initializing GUI node ...")
        print("Init")

        init_msgs(self)


        self.model = Model()


    






        


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    #############################################################
    # INITIALIZATION
    #############################################################

    signal.signal(signal.SIGINT, sig_handler)
    rospy.init_node('GUI')   

    time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

    node = GuiNode()
    

    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__ == '__main__':
    main()
