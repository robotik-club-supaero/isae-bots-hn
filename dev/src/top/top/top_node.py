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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import socket
from std_msgs.msg      import Int16, Empty, Int16MultiArray

from top_const import *

#######################################################################
# TOP NODE
#######################################################################


class TopNode(Node):
    """
    TOP node.
    """
 
    topServerSocket = None

    def __init__(self):	
        super().__init__("TOP")
        self.get_logger().info("Initializing TOP Node ...")

        latch_profile =  QoSProfile(
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Transient Local durability
        )

        # --- Publishers & subscribers
        self.pubStart = self.create_publisher(Int16, "/game/start",latch_profile)
        self.pubColor = self.create_publisher(Int16, "/game/color", latch_profile)
        self.pubBRIdle = self.create_publisher(Int16, "/br/idle", latch_profile)
        
        self.subIsbMatch = self.create_subscription(Int16, "/okPosition", self.callBackBR, 10)
        
        # client socket
        self.topServerSocket = socket.socket()
        self.topServerSocket.connect( ('127.0.0.1', TOPSERVER_PORT))

        self.get_logger().info("TOP node is ready")



    # --- Callback functions

    def callBackBR(self, msg):

        # if msg.data == 5:  # OK_READY
        #     self.writeLed(BR_IDLE_LED_ID, 1)

        # elif msg.data == 6:  # OK_IDLE
        #     self.writeLed(BR_IDLE_LED_ID, 0)
        
        res = self.topServerSocket.recv(1024).decode()
        
        return


    def close(self):
        
        self.topServerSocket.close()
        

#######################################################################
# LAUNCH
#######################################################################

def main():
    rclpy.init(args=sys.argv)
    
    node = TopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()