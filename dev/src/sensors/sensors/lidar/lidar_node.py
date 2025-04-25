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

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from br_messages.msg import Position
from message.msg import ProximityMap
from config.qos import default_profile, br_position_topic_profile

from .lidar_config import *

class LidarNode(Node):

    def __init__(self):
        super().__init__('LidarNode')
        self.get_logger().info("Initializing Lidar Node.")

        self.robot_pos = Position()
        self.obstacle_msg = ProximityMap()
        self.obstacle_msg.source = ProximityMap.LIDAR

        # initialisation des publishers
        self.pub_obstacles = self.create_publisher(ProximityMap, "/sensors/obstacles", default_profile)
        
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

        self.obstacle_msg.cluster_dists.clear()
        self.obstacle_msg.x_r.clear()
        self.obstacle_msg.y_r.clear()
        self.obstacle_msg.cluster.clear()

        for i, cluster in enumerate(clusters):
            self.obstacle_msg.cluster_dists.append(cluster.dist)

            for point in cluster.points:
                self.obstacle_msg.x_r.append(point.x)
                self.obstacle_msg.y_r.append(point.y)
                self.obstacle_msg.cluster.append(i)

        self.pub_obstacles.publish(self.obstacle_msg)
        
    def _get_coords(self, msg):
        # See https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        range_min = max(MIN_RANGE, msg.range_min)
        range_max = min(MAX_RANGE, msg.range_max)

        if DROP_OFF_LIMITS:
            x_r = self.robot_pos.x
            y_r = self.robot_pos.y
            theta_r = self.robot_pos.theta
        
        theta = angle_min
        obstList = [] 
        for dist in ranges:
            # On applique un masque pour supprimer les points qui ne sont pas dans les bornes indiquées (bornes de détection du LiDAR).
            if range_min<dist<range_max:
                if DROP_OFF_LIMITS:
                    # Calcul des coords absolue sur la table
                    x_abs = x_r + 1000*dist*math.cos(theta_r + theta)
                    y_abs = y_r + 1000*dist*math.sin(theta_r + theta)
                    # On supprime les points en dehors de la table.
                    if not ((0 < y_abs < TABLE_H) and (0 < x_abs < TABLE_W)):
                        continue
            
                # Conversion de (d, theta) en (x_r, y_r)
                x_rel = 1000*dist*math.cos(theta)
                y_rel = 1000*dist*math.sin(theta)
                obstList.append(RawPoint(x_rel, y_rel, dist, theta))
                
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

                if abs(point.theta - current_cluster.lastTheta) > MIN_ANGLE_RESOLUTION:
                    # Same cluster, but too far from the last retained point of the cluster (a wide cluster should keep multiple points for accuracy)
                    current_cluster.append(point)
                # else:
                    # Close enough to the last retained point of the cluster. Just drop the point.
            
            else:
                # Different cluster
                current_cluster.append(lastPos)
                clusters.append(current_cluster.build())

                current_cluster = ClusterBuilder(point)

            lastPos = point
        
        if clusters != [] and lastPos.euclidean_distance(clusters[0].points[0]) < CLUSTER_DIST_LIM:
            # First and last clusters are the same
            # This can happen on 360° LiDAR, where the last angle in the list is actually close to the first one.
            clusters[0] = clusters[0].combine(current_cluster.build())
        else:
            current_cluster.append(lastPos)
            clusters.append(current_cluster.build())

        return clusters
        
class ClusterBuilder:
    def __init__(self, pt):
        self._points = []
        self._lastTheta = None
        self._dist = None

        self.append(pt)

    def append(self, pt):
        x, y, dist, theta = pt.x, pt.y, pt.dist, pt.theta

        if theta == self.lastTheta:
            return

        self._points.append((x,y))
        self._lastTheta = theta
        if self._dist is None or dist < self._dist:
            self._dist = dist

    def build(self):
        return Cluster(self._points, self._dist)

    @property
    def lastTheta(self):
        return self._lastTheta

    @property
    def dist(self):
        return self._dist

    @property
    def points(self):
        return self._points

@dataclass(frozen=True)
class Cluster:
    points: list
    dist: float

    def combine(self, other):
        points, dist = self.points, self.dist
        points.extend(other.points)
        dist = min(dist, other.dist)

        return Cluster(points, dist)

class RawPoint:
    """Before clustering"""

    def __init__(self, x, y, dist, theta):
        self.x = x
        self.y = y
        self.dist = dist
        self.theta = theta

    def euclidean_distance(self, pt2):
        """Retourne la distance entre 2 points dans un repère cartésien. Ici, le repère du robot."""
        return math.sqrt((self.x - pt2.x)**2 + (self.y - pt2.y)**2)
    