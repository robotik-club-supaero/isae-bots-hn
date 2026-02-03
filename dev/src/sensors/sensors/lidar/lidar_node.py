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
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan

from br_messages.msg import Position
from message_utils.geometry import make_absolute
from message.msg import SensorObstacleList, SensorObstacle
from config.qos import default_profile, br_position_topic_profile

from .lidar_config import *

class LidarNode(Node):

    def __init__(self):
        super().__init__('LidarNode')
        self.get_logger().info("Initializing Lidar Node.")

        self.robot_pos = Position()
        self.obstacle_msg = SensorObstacleList()

        # initialisation des publishers
        self.pub_obstacles = self.create_publisher(SensorObstacleList, "/sensors/obstaclesLidar", default_profile)
        
        # initialisation des suscribers
        if DROP_OFF_LIMITS:
            self.sub_pos = self.create_subscription(Position, "/br/currentPosition", self.update_position, br_position_topic_profile)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.update_obstacle, default_profile)

        self.get_logger().info("Lidar Node initialized")

    def update_position(self, msg):
        self.robot_pos = msg

    def update_obstacle(self, msg):
        obstacleCoords = self._get_coords(msg)
        clusters = self._clustering(obstacleCoords)

        self.obstacle_msg.obstacles.clear()

        for cluster in clusters:
            if cluster.num_of_points >= DETECTION_THRESHOLD:
                point = cluster.build()
                self.obstacle_msg.obstacles.append(SensorObstacle(x=point.x, y=point.y))

        self.pub_obstacles.publish(self.obstacle_msg)
        
    def _get_coords(self, msg):
        # See https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        range_min = max(MIN_RANGE, 1000*msg.range_min)
        range_max = min(MAX_RANGE, 1000*msg.range_max)

        if DROP_OFF_LIMITS:
            cos = math.cos(self.robot_pos.theta)
            sin = math.sin(self.robot_pos.theta)
        
        theta = angle_min
        obstList = [] 
        for dist in ranges:
            dist_mm = dist * 1000

            # On applique un masque pour supprimer les points qui ne sont pas dans les bornes indiquées (bornes de détection du LiDAR).
            if range_min<dist_mm<range_max:
                # Conversion de (d, theta) en (x_rel, y_rel)
                x_rel = dist_mm * math.cos(theta - LIDAR_ANGLE*math.pi/180)  + LIDAR_OFFSET_X
                y_rel = dist_mm * math.sin(theta - LIDAR_ANGLE*math.pi/180) + LIDAR_OFFSET_Y
                obstacle = Point(x_rel, y_rel)

                if DROP_OFF_LIMITS:
                    x_abs, y_abs = make_absolute(self.robot_pos, obstacle, cos=cos, sin=sin)

                # On supprime les points en dehors de la table. 
                if not DROP_OFF_LIMITS or ((0 < y_abs < TABLE_H) and (0 < x_abs < TABLE_W)):                    
                    obstList.append(obstacle)

            theta += angle_inc

        return obstList

    def _clustering(self, coords):
        """
        Fonction permettant de clusteriser les points obtenus après traitement. Cela diminue le nombre de points et identifie les obstacles.
        Les points sont, par construction, dans le sens de l'augmentation de l'angle theta du LiDAR. 
        """

        if coords == []:
            return []

        clusters = []

        lastPos = coords[0]
        current_cluster = ClusterBuilder(coords[0])

        for point in coords[1:]:
            if lastPos.euclidean_distance(point) < CLUSTER_DIST_LIM:
                # On regarde si le point suivant fait partie du regroupement en regardant sa distance au regroupement.
                current_cluster.append(point)
            
            else:
                # Different cluster
                clusters.append(current_cluster)

                current_cluster = ClusterBuilder(point)

            lastPos = point
        
        if clusters != [] and lastPos.euclidean_distance(coords[0]) < CLUSTER_DIST_LIM:
            # First and last clusters are the same
            # This can happen on 360° LiDAR, where the last angle in the list is actually close to the first one.
            clusters[0].combine(current_cluster)
        else:
            clusters.append(current_cluster)

        return clusters
        
class ClusterBuilder:
    def __init__(self, pt):
        self._x = 0
        self._y = 0
        self._len = 0

        self.append(pt)

    def append(self, pt):
        self._x += pt.x
        self._y += pt.y
        self._len += 1

    def combine(self, other):
        self._x += other._x
        self._y += other._y
        self._len += other._len

    def build(self):
        return Point(self._x / self._len, self._y / self._len)

    @property
    def num_of_points(self):
        return self._len

@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def euclidean_distance(self, pt2):
        """Retourne la distance entre 2 points dans un repère cartésien. Ici, le repère du robot."""
        return math.sqrt((self.x - pt2.x)**2 + (self.y - pt2.y)**2)
    

#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
