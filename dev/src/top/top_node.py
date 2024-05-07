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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import sys
from time import perf_counter, sleep
import rospy
import signal
import socket
from std_msgs.msg      import Int16, Empty, Int16MultiArray
from geometry_msgs.msg import Quaternion

from top_const import *

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[TOP]"



def log_info(msg):
    """
    Print logs standard.
    """
    rospy.loginfo(f"{_NODENAME_} {msg}")


def log_warn(msg):
    """
    Print logs warning.
    """
    rospy.logwarn(f"{_NODENAME_} {msg}")


def log_errs(msg):
    """
    Print logs errors.
    """
    rospy.logerr(f"{_NODENAME_} {msg}")


def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    log_warn("Node forced to terminate ...")
    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()



#######################################################################
# TOP NODE
#######################################################################


class TopNode:
    """
    TOP node.
    """
 
    topServerSocket = None

    def __init__(self):	
        log_info("Initializing TOP Node ...")



        # --- Publishers & subscribers
        self.pubStart = rospy.Publisher("/game/start", Int16, queue_size=10, latch=True)
        self.pubColor = rospy.Publisher("/game/color", Int16, queue_size=10, latch=True)
        self.pubBRIdle = rospy.Publisher("/br/idle", Int16, queue_size=10, latch=True)
        
        self.subIsbMatch = rospy.Subscriber("/okPosition", Int16, self.callBackBR)
        
        
        # client socket
        self.topServerSocket = socket.socket()
        self.topServerSocket.connect( ('127.0.0.1', TOPSERVER_PORT))


        log_info("TOP node is ready")



    # --- Callback functions

    def callBackBR(self, msg):

        # if msg.data == 5:  # OK_READY
        #     self.writeLed(BR_IDLE_LED_ID, 1)

        # elif msg.data == 6:  # OK_IDLE
        #     self.writeLed(BR_IDLE_LED_ID, 0)
        
        res = self.topServerSocket.recv(1024).decode()
        
        return



    def run(self):

        while True:

            if perf_counter() - self.currentTime > 0.01:
                pass

            sleep(0.001)
            
    def close(self):
        
        self.topServerSocket.close()
        

#######################################################################
# LAUNCH
#######################################################################

def main():
    rospy.init_node('isb_node')
    signal.signal(signal.SIGINT, sig_handler)

    node = TopNode()

    # main loop
    node.run()

    rospy.spin()


if __name__ == '__main__':
    main()