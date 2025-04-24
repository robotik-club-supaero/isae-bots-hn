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
import sys
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from br_messages.msg import Position
from config.qos import default_profile, br_position_topic_profile

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

OBSTACLE_RADIUS = 150 # should match the plot radius defined in the interface for consistent display
SPEED = 200 # mm/s | can be 0 for fixed obstacle

def oscillationY(ti, ymin, ymax, w, phi):
	return (ymin + ymax)/2 + (ymax - ymin)/2 * np.sin(w * (time.time() - ti) + phi)

def oscillationX(ti, xmin, xmax, w, phi):
	return (xmin + xmax)/2 + (xmax - xmin)/2 * np.sin(w * (time.time() - ti) + phi)

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

        self.position_sub = self.create_subscription(Position, "/br/currentPosition", self.recv_position, br_position_topic_profile)
        self.obs_info_pub = self.create_publisher(Int16MultiArray, "/obstaclesInfo", default_profile)

        self.create_time = time.time()
        
        self.get_logger().info("OBS node initialized")

    def seen_obstacle(self, nbr=0):
        """
        Generates positions for fake obstacles.
        """
        # NB: try and upgrade this function for more complex examples...
        # obs_positions = [(1468,oscillationY(self.ti, 1800, 2500, 1, 0)),(oscillationX(self.ti, 1000, 1500, 1, 0),1550)]
        obs_positions = [(1200, 600)]
        return obs_positions
    
    def generate_fix_obstacle_data(self, x, y):
        obstacles_pos = [(x, y, np.linalg.norm([self.x_robot-x, self.y_robot-y]), 0, 0)]
        
        return obstacles_pos
    

    def recv_position(self, msg):
        """
        Feedback of obstacles positions for simulation
        """
        ###############################################################
        ## Info about the robot
        ###############################################################
        self.x_robot = msg.x
        self.y_robot = msg.y
        self.cap = msg.theta

        ###############################################################
        ## Make the info msg to send
        ###############################################################
        
        dist = (1000 + SPEED * (time.time() - self.create_time)) % 4000
        if dist >= 2000:
            x = 4000 - dist
        else:
            x = dist
        obstacle_data = self.generate_fix_obstacle_data(x, 2000)

        data = [0]
        for pos in obstacle_data:
            for coord in pos:
                data.append((int)(coord))

        dim1 = MultiArrayDimension()
        dim1.label = "nbObstacles"
        dim1.size = len(obstacle_data)
        dim1.stride = 5* len(obstacle_data)

        dim2 = MultiArrayDimension()
        dim2.label = "coordinates"
        dim2.size = 5
        dim2.stride = 5

        dimensions = []
        dimensions.append(dim1)
        dimensions.append(dim2)

        layout = MultiArrayLayout()
        layout.data_offset = 1
        layout.dim = dimensions

        newmsg = Int16MultiArray()
        newmsg.data = data
        newmsg.layout = layout

        self.obs_info_pub.publish(newmsg)


#######################################################################
#																      #
# 								MAIN	 						      #
#																      #
#######################################################################	

def main():
    rclpy.init(args=sys.argv)
    
    node = SIM_ObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
