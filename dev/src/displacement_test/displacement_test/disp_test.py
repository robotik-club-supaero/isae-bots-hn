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
        self.round = max(NB_TOURS, 1)

        # initialisation des publishers
        self.pub_dispOrd = self.create_publisher(DisplacementOrder, '/br/goTo', latch_profile)
        self.pub_idle = self.create_publisher(Bool, '/br/idle', default_profile)
        
        # initialisation des suscribers
        self.sub_callback = self.create_subscription(Int16, '/br/callbacks', self._launchNewRound, default_profile)

        self.launch = Int16()
        self.launch.data = 7
        self.get_logger().info("Displacement Test Node initialized")
        self._launchNewRound(self.launch)

    def _launchNewRound(self, msg):
        if not(self.idle.data):
            self._changeIdle(run=True)
        elif ((msg.data == 7) and (self.round >= 1)):
            #msg.data == 7 => Déplacement fini
            self.round = self.round-1
            self.pub_dispOrd.publish(self.dispOrder)
        elif (msg.data == 6):
            #msg.data == 6 => okIdle, contrôleur passé en état Idle
            self.idle.data = False
        elif (msg.data == 5):
            #msg.data == 5 => okReady, contrôleur passé en état Ready
            self._launchNewRound(self.launch)
        elif (self.round == 0):
            self.get_logger().warning("Node ended successfully !")
            self._changeIdle(False)
            self.destroy_node()
            rclpy.try_shutdown()

    def _changeIdle(self, run=True):
        self.idle.data = run
        self.pub_idle.publish(self.idle)
        if (run == True):
            self.get_logger().warning("Entering running mode")
        else:
            self.get_logger().warning("Entering idle mode")
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
