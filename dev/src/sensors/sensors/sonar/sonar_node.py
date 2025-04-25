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


#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import sys

import rclpy
from rclpy.node import Node
import numpy as np
import ast
from sonar_lib import coord_obstacle
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Point
from br_messages import Position
from config import SonarConfig
from config.qos import default_profile, br_position_topic_profile

#################################################################
#                                                               #
#                         SONAR node                            #
#                                                               #
#################################################################

# OBSOLETE!!!!!!!!!
class SonarNode(Node):
    """
    ROS node SONAR node for sonar obstacles detection.
    """

    def __init__(self):
        super().__init__("SON")
        self.get_logger().info("Initializing SonarNode ...")
        
        # Get sonars from config file of the robot
        config = SonarConfig()
        
        self.nb_sonars = 0
        self.sonars_lst = []     #on enregistre chaque sonar (sa position et son cap) dans cette liste
        self.sonars_pos = []
        
        for sonar in config.available_sonars:
            self.sonars_lst.append(sonar)
            self.sonars_pos.append([0 for i in range(3)])
            self.nb_sonars += 1

        self.obs_pub = self.create_publisher(Int16MultiArray, "/sensors/obstaclesSonar", default_profile)
        self.pos_sub = self.create_subscription(Position, "/br/currentPosition", self.recv_position, br_position_topic_profile)
        self.son_sub = self.create_subscription(Point, "/ultrasonicDistances", self.recv_obstacle, default_profile)  # can change topic name ?
    
    def recv_position(self, msg):
        """
        Feedback on current position /br/currentPosition topic.
        """
        for i in range(self.nb_sonars) :
            [x, y, dir_visee] = self.sonars_lst[i]
            pol_dst = np.hypot(x, y)
            pol_cap = np.arctan2(y, x)
            dir_visee = dir_visee * np.pi/180
            self.sonars_pos[i][0] = msg.x + pol_dst * np.cos(msg.theta + pol_cap)
            self.sonars_pos[i][1] = msg.y + pol_dst * np.sin(msg.theta + pol_cap)
            self.sonars_pos[i][2] = msg.theta + dir_visee

        self.x_robot = msg.x
        self.y_robot = msg.y
        self.cap = msg.theta

    def recv_obstacle(self, msg):
        """
        Feedback on obstacle info from sonars
        """
        obs_lst = []
        dst_lst = [msg.x, msg.y]

        for i in range(self.nb_sonars):
            dst = dst_lst[i]
            pos = self.sonars_pos[i]
            coord = coord_obstacle(pos, dst)
            if coord : obs_lst.append(coord)

        info = Int16MultiArray()
        data = [1]
        for pos in obs_lst:
            for coord in pos:
                data.append((int)(coord))
        info.data = data

        layout = MultiArrayLayout()
        layout.data_offset = 1

        dims = []
        dim1 = MultiArrayDimension()
        dim1.label = "nbObstacles"
        dim1.size = len(obs_lst)
        dim1.stride = 2* len(obs_lst)

        dim2 = MultiArrayDimension()
        dim2.label = "coordinates"
        dim2.size = 2
        dim2.stride = 2

        dims.append(dim1)
        dims.append(dim2)
        layout.dim = dims
        info.layout = layout

        self.obs_pub.publish(info)


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = ObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
