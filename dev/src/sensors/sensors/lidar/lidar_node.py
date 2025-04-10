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

"""
@file: LidarNode.py
@status: OK

Fichier de gestion des obstacles LIDAR
"""

import sys
from math import atan2, sin, cos
from lidar_lib import *

import rclpy
from rclpy.node import Node
from br_messages.msg import Position
from sensor_msgs.msg   import LaserScan
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Int16

from config.qos import default_profile, br_position_topic_profile

OBS_RESOLUTION = 100

#######################################################################
# LIDAR NODE
#######################################################################

class LidarNode(Node):

    """LidarNode class"""

    def __init__(self):
        """Initialisation."""
        ### VARIABLES #################################################
        self.x_robot = 0
        self.y_robot = 0
        self.c_robot = 0
        self.radius = 42

        self.iterData = 0
        self.sizeData = 5
        self.obstaclesLists = [ [] for i in range(self.sizeData)]

        ### LAUNCH NODE ###############################################
        super().__init__('LidarNode')
        self.get_logger().info("Initializing Lidar Node.")

        # initialisation des publishers
        self.pub_obstacles = self.create_publisher(Int16MultiArray, "/sensors/obstaclesLidar", default_profile)
        # initialisation des suscribers
        self.sub_pos = self.create_subscription(Position, "/br/currentPosition", self.update_position, br_position_topic_profile)
        self.sub_hokuyo = self.create_subscription(LaserScan, "/scan", self.update_obstacle, default_profile)


    def update_position(self,msg):
        """Fonction de callback de position."""
        self.x_robot = msg.x
        self.y_robot = msg.y
        self.c_robot = msg.theta
 
    def update_obstacle(self, msg):
        """Fonction de callback des obstacles lidar."""
        x_r = self.x_robot
        y_r = self.y_robot
        cap = self.c_robot

        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment  
        range_min = msg.range_min
        range_max = msg.range_max
        
        raw_data = lidar_to_table(x_r, y_r, cap, ranges, angle_min, angle_max, angle_inc, range_min, range_max )
        obs_detected = clusterisation(raw_data)
        self.obstaclesLists[self.iterData % self.sizeData] = obs_detected
        self.iterData += 1
        self.treats_obstacle()

    def treats_obstacle(self): # TODO - 
        """Fonction appelee pour eliminer les 'doublons'.
        
        Cette fonction va comparer les obstacles detectes au temps
        t-1 et ceux au temps t. Elle determine alors si un obstacle du 
        temps t est en realite le meme obstacle du temps t-1 qui s'est 
        deplace. 
        
        Pour cela, on regarde si pour chaque obstacle au temps t, il y 
        a des obstacles au temps t-1 qui lui sont assez proche."""
        
        obstaclesLists = list(self.obstaclesLists)

        indexOfMax = 0
        maxLen = len(obstaclesLists[0])
        for i in range(1, self.sizeData):
            if(len(obstaclesLists[i]) > maxLen):
                indexOfMax = i
                maxLen = len(obstaclesLists[i])

        biggestList = obstaclesLists[indexOfMax]

        calculatedObstacles = []
        for potentialObstacle in biggestList :
            nbOcc = 1
            for i in range(self.sizeData):
                if( i != indexOfMax):
                    for obstacle in obstaclesLists[i]:
                        if(euclidean_distance(potentialObstacle, obstacle) < OBS_RESOLUTION):
                            nbOcc += 1
            if(nbOcc > 2):
                x_robot = self.x_robot
                y_robot = self.y_robot
                x = potentialObstacle[0]
                y = potentialObstacle[1]
                theta = atan2((y-y_robot),(x-x_robot))

                calculatedObstacles.append([x - self.radius * cos(theta),  y - self.radius * sin(theta)])

        msg = Int16MultiArray()

        data = [0] # Le 0 est l'identifiant du LiDAR dans le SensorsNode.
        for position in calculatedObstacles:
            for coordinate in position:
                data.append((int)(coordinate))

        msg.data = data

        layout = MultiArrayLayout()
        layout.data_offset = 1

        dimensions = []
        dim1 = MultiArrayDimension()
        dim1.label = "obstacle_nb"
        dim1.size = len(calculatedObstacles)
        dim1.stride = 2* len(calculatedObstacles)

        dim2 = MultiArrayDimension()
        dim2.label = "coordinate"
        dim2.size = 2
        dim2.stride = 2

        dimensions.append(dim1)
        dimensions.append(dim2)

        layout.dim = dimensions

        msg.layout = layout

        self.pub_obstacles.publish(msg)



#######################################################################
# MAIN
#######################################################################

def main():
    rclpy.init(args=sys.argv)

    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()