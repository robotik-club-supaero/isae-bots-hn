#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false
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

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from br_messages.msg import DisplacementOrder, Point
from std_msgs.msg import Int16, Bool
from config.qos import default_profile, latch_profile

from .test_config import *

class DispTestNode(Node):
 
    def __init__(self):
        super().__init__('DispTestNode')
        self.get_logger().info("Initializing Displacement Test Node.")

        self.dispOrder = DisplacementOrder()
        self.dispOrder._path = [Point(x=point[0], y=point[1]) for point in PATH]
        self.dispOrder._kind = KIND
        self.dispOrder._theta = THETA
        self.idle = Bool()
        self.idle.data = False

        if (NB_TOURS > 1):
            self.round = NB_TOURS
        else:
            self.round = 1

        # initialisation des publishers
        self.pub_dispOrd = self.create_publisher(DisplacementOrder, '/br/goTo', latch_profile)
        self.pub_idle = self.create_publisher(Bool, '/br/idle', default_profile)
        
        # initialisation des suscribers
        self.sub_callback = self.create_subscription(Int16, '/br/callbacks', self._launchNewRound, default_profile)

        launch = Int16()
        launch.data = 7
        self._launchNewRound(launch)
        self.get_logger().info("Displacement Test Node initialized")

    def _launchNewRound(self, msg):
        if not(self.idle.data):
            self._setReady()
        if ((msg.data == 7) & (self.round > 0)):
            #msg.data == 7 => Déplacement fini
            self.round = self.round - 1
            self.pub_dispOrd.publish(self.dispOrder)
        elif (msg == 6):
            self.idle.data = False

    def _setReady(self):
        self.idle.data = True
        self.pub_idle.publish(self.idle)
        time.sleep(2)

#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)
    node = DispTestNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
