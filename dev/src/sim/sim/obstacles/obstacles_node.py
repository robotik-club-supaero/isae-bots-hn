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
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import time
import random
import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

from message.msg import ProximityMap
from br_messages.msg import Position, Point

from config import RobotConfig
from config.qos import default_profile, br_position_topic_profile

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

OBSTACLE_RADIUS = 150 # should match the plot radius defined in the interface for consistent display
SPEED = 200 # mm/s | can be 0 for fixed obstacle

#################################################################
#                                                               #
#                        OBSTACLES NODE                         #
#                                                               #
#################################################################

class SIM_ObstaclesNode(Node):
    """
    SIM OBS node: obstacles ros node for simulation.
    """

    def __init__(self):
        super().__init__("OBS")
        self.get_logger().info("Initializing OBS node ...")

        config = RobotConfig()
        self.robot_diag = config.robot_diagonal

        self.robot_pos = np.array([0., 0.])
        self.robot_theta = 0
        self.obstacle_pos = np.array([1000., 2000.])
        self.obstacle_speed = None
        
        self.position_sub = self.create_subscription(Position, "/br/currentPosition", self.recv_position, br_position_topic_profile)
        self.obs_info_pub = self.create_publisher(ProximityMap, "/sensors/obstacles", default_profile)
        self.obs_simu_pub = self.create_publisher(Int16MultiArray, "/simu/robotObstacle", default_profile)
        
        self.get_logger().info("OBS node initialized")

    def recv_position(self, msg):
        self.robot_pos[0] = msg.x
        self.robot_pos[1] = msg.y
        self.robot_theta = msg.theta

    def _shouldChangeSpeed(self):
        if self.obstacle_speed is None:
            return True # Should set initial speed

        if random.random() > 0.999:
            return True # Randomly change speed

        # Stay on the table
        if self.obstacle_pos[0] < OBSTACLE_RADIUS and self.obstacle_speed[0] < 0:
            return True
        if self.obstacle_pos[0] > 2000-OBSTACLE_RADIUS and self.obstacle_speed[0] > 0:
            return True
        if self.obstacle_pos[1] < OBSTACLE_RADIUS and self.obstacle_speed[1] < 0:
            return True
        if self.obstacle_pos[1] > 3000-OBSTACLE_RADIUS and self.obstacle_speed[1] > 0:
            return True

        dist = np.linalg.norm(self.robot_pos - self.obstacle_pos)
        if dist < self.robot_diag + OBSTACLE_RADIUS and np.dot(self.obstacle_speed, self.robot_pos-self.obstacle_pos) > 0:
            return True

        return False

    def run(self):
       
        msg = ProximityMap()
        msg.source = ProximityMap.SIM_OBS_NODE
        msg.cluster = [0]

        t = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            dist = np.linalg.norm(self.robot_pos - self.obstacle_pos)

            n = time.time()
            elapsed = n - t
            t = n
            
            while self._shouldChangeSpeed():
                # Change speed randomly or to avoid opponent                        
                speed_vec = np.random.rand(2) * 2 * SPEED - SPEED
                speed = np.linalg.norm(speed_vec)

                if speed > SPEED:
                    speed_vec *= SPEED / speed
                elif speed < SPEED * 0.8:
                    speed_vec *= SPEED / speed * 0.8

                self.obstacle_speed = speed_vec

            self.obstacle_pos += self.obstacle_speed * elapsed
             
            dist = np.linalg.norm(self.robot_pos - self.obstacle_pos)
            x_r, y_r = make_relative(self.robot_pos, self.robot_theta, self.obstacle_pos)

            msg.cluster_dists = [max(0, dist-OBSTACLE_RADIUS)]
            msg.x_r = [x_r]
            msg.y_r = [y_r]

            self.obs_info_pub.publish(msg)
            self.obs_simu_pub.publish(Int16MultiArray(data=[int(self.obstacle_pos[0]), int(self.obstacle_pos[1]), OBSTACLE_RADIUS]))

def make_relative(robot, robot_theta, obstacle):
    cos = math.cos(robot_theta)
    sin = -math.sin(robot_theta)

    vector = obstacle - robot
    return (vector[0] * cos - vector[1] * sin, vector[0] * sin + vector[1] * cos)

#######################################################################
#																      #
# 								MAIN	 						      #
#																      #
#######################################################################	

def main():
    rclpy.init(args=sys.argv)
    
    node = SIM_ObstaclesNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
