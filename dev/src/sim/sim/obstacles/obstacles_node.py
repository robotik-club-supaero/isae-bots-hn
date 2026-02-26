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
from rclpy.executors import ExternalShutdownException

from message.msg import SensorObstacleList, SensorObstacle, CircleObstacle
from message_utils.geometry import make_relative
from br_messages.msg import Position, Point

from config import RobotConfig, NaiveStratConfig
from config.qos import default_profile, br_position_topic_profile

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

OBSTACLE_MODE = NaiveStratConfig().enable_obstacle_sim
OBSTACLE_RADIUS = 150 # should match the plot radius defined in the interface for consistent display
SPEED = 500 # mm/s | can be 0 for fixed obstacle
INIT_POS = [500., 1300.]

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
        self.robot_diag = config.robot_diagonal / 2

        self.robot_pos = np.array([0., 0.])
        self.robot_theta = 0
        self.obstacle_pos = np.array(INIT_POS)
        self.obstacle_speed = None
        self.last_update = None
        
        self.position_sub = self.create_subscription(Position, "/br/currentPosition", self.recv_position, br_position_topic_profile)
        self.obs_info_pub = self.create_publisher(SensorObstacleList, "/sensors/obstaclesLidar", default_profile)
        self.obs_simu_pub = self.create_publisher(CircleObstacle, "/simu/robotObstacle", default_profile)
        self.update_timer = self.create_timer(0.01, self.update_pos)

        self.msg = SensorObstacleList()
        
        self.get_logger().info("OBS node initialized")

    def recv_position(self, msg):
        self.robot_pos[0] = msg.x
        self.robot_pos[1] = msg.y
        self.robot_theta = msg.theta

    def _shouldChangeSpeed(self):
        if self.obstacle_speed is None:
            return True # Should set initial speed

        if random.random() > 0.9999:
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
        if dist < self.robot_diag + OBSTACLE_RADIUS + 20 and np.dot(self.obstacle_speed, self.robot_pos-self.obstacle_pos) > 0:
            self.obstacle_speed[:] = 0

        # L'obstacle ne bouge pas quand il est proche de notre robot afin de tester nos manoeuvres d'évitement/éloignement
        return SPEED > 0 and (dist > self.robot_diag + OBSTACLE_RADIUS + 100 or random.random() > 0.99) and np.linalg.norm(self.obstacle_speed) == 0
        
    def update_pos(self):
        if self.last_update is None: self.last_update = time.time()
      
        dist = np.linalg.norm(self.robot_pos - self.obstacle_pos)

        n = time.time()
        elapsed = n - self.last_update
        self.last_update = n
        
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
            
        x_rel, y_rel = make_relative(Position(x=self.robot_pos[0], y=self.robot_pos[1], theta=self.robot_theta), Point(x=self.obstacle_pos[0], y=self.obstacle_pos[1]))

        self.msg.obstacles = [SensorObstacle(x=x_rel, y=y_rel)]
        self.obs_info_pub.publish(self.msg)
        self.obs_simu_pub.publish(CircleObstacle(x=self.obstacle_pos[0], y=self.obstacle_pos[1], radius=float(OBSTACLE_RADIUS)))

#######################################################################
#																      #
# 								MAIN	 						      #
#																      #
#######################################################################	

def main():

    if not OBSTACLE_MODE:
        return

    rclpy.init(args=sys.argv)
    
    node = SIM_ObstaclesNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
