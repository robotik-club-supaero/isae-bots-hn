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

from .lidar_positioning_config import *

class LidarPositioningNode(Node):
 
    def __init__(self):
        super().__init__('LidarPositioningNode')
        self.get_logger().info("Initializing Lidar Positioning Node.")

        self.robot_pos = Position()
        self.obstacle_msg = SensorObstacleList()

        self.mark_1 = Point(x=MARK_1_X, y=MARK_1_Y)
        self.mark_2 = Point(x=MARK_2_X, y=MARK_2_Y)
        self.mark_3 = Point(x=MARK_3_X, y=MARK_3_Y)

        self.dist_12 = self.mark_1.euclidean_distance(self.mark_2)
        self.dist_13 = self.mark_1.euclidean_distance(self.mark_3)
        self.dist_23 = self.mark_2.euclidean_distance(self.mark_3)

        self.mark_1_pos_rel = Point(x=0., y=0.)
        self.mark_2_pos_rel = Point(x=0., y=0.)
        self.mark_3_pos_rel = Point(x=0., y=0.)

        # initialisation des publishers
        self.pub_obstacles = self.create_publisher(SensorObstacleList, "/sensors/obstaclesLidar", default_profile)
        self.pub_position = self.create_publisher(Position, "/br/currentPosition", br_position_topic_profile)
        
        # initialisation des suscribers
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.update_positions, default_profile)

        self.get_logger().info("Lidar Positioning Node initialized")

    def update_positions(self, msg):
        #On calcule la position relative de chaque point
        obstacleCoords = self._get_coords(msg)

        #On les regroupe en cluster
        clusters_unfiltered = self._clustering(obstacleCoords)

        #On filtre les clusters selon leur taille
        clusters = []

        for cluster in clusters_unfiltered:
            if cluster.num_of_points >= DETECTION_THRESHOLD:
                clusters.append(cluster.build())

        #On cherche parmi les clusters la position des trois repères
        marks = self._get_marks(clusters)

        #On déduit la position du robot de la position des repères
        #On passe les coordonnées dans le repère absolu
        #On filtre les points hors de la table
        #On publie la position du robot, des repères et des obstacles

        self.obstacle_msg.obstacles.clear()

        for cluster in clusters:
            if cluster.num_of_points >= DETECTION_THRESHOLD:
                point = cluster.build()
                self.obstacle_msg.obstacles.append(SensorObstacle(x=point.x, y=point.y))

        self.pub_obstacles.publish(self.obstacle_msg)

    def _get_marks(self, clusters):
        """ 
        Parmi les clusters, on cherche les trois points qui correspondent à la position des repères
	        Si deux sets ou plus fonctionnent, on prend celui le plus proche des repères précédents
	        Si aucun set ne convient, on test avec seulement deux points. De même que précédemment si plusieurs sets conviennent
	        Si aucun set de deux points ne convient non plus, on log une erreur et on attend le prochain scan 
        """
        
        valid_marks = []

        for i in range(len(clusters)-1):
            for j in range(i+1, len(clusters)):
                if (abs(clusters[i].euclidean_distance(clusters[j]) - self.dist_12) < EPS ):
                    for k in range(len(clusters)):
                        if ((k != i) & (k != j)):
                            dist_ik = clusters[i].euclidean_distance(clusters[k])
                            dist_jk = clusters[j].euclidean_distance(clusters[k])
                            if (((abs(dist_ik - self.dist_13) < EPS) & (abs(dist_jk - self.dist_23) < EPS)) \
                                        | ((abs(dist_ik - self.dist_23) < EPS) & (abs(dist_jk - self.dist_13) < EPS))):
                                valid_marks.append([clusters[i], clusters[j], clusters[k]])
                        
        if (len(valid_marks == 1)):
            return valid_marks[0]
        elif (len(valid_marks) > 1):
            i_min = 0
            min_dist = 10000000
            for j in range(len(valid_marks)):
                marks = valid_marks[j]
                dist_old_marks = min(marks[0].euclidean_distance(self.mark_1_pos_rel) + marks[1].euclidean_distance(self.mark_2_pos_rel) + marks[2].euclidean_distance(self.mark_3_pos_rel), \
                                     marks[0].euclidean_distance(self.mark_1_pos_rel) + marks[2].euclidean_distance(self.mark_2_pos_rel) + marks[1].euclidean_distance(self.mark_3_pos_rel), \
                                     marks[1].euclidean_distance(self.mark_1_pos_rel) + marks[0].euclidean_distance(self.mark_2_pos_rel) + marks[2].euclidean_distance(self.mark_3_pos_rel), \
                                     marks[1].euclidean_distance(self.mark_1_pos_rel) + marks[2].euclidean_distance(self.mark_2_pos_rel) + marks[0].euclidean_distance(self.mark_3_pos_rel), \
                                     marks[2].euclidean_distance(self.mark_1_pos_rel) + marks[0].euclidean_distance(self.mark_2_pos_rel) + marks[1].euclidean_distance(self.mark_3_pos_rel), \
                                     marks[2].euclidean_distance(self.mark_1_pos_rel) + marks[1].euclidean_distance(self.mark_2_pos_rel) + marks[0].euclidean_distance(self.mark_3_pos_rel), \
                )
                if (dist_old_marks < min_dist):
                    i_min = j
                    min_dist = dist_old_marks
            return valid_marks[i_min]


        return valid_marks

    def _get_coords(self, msg):
        # See https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        range_min = max(MIN_RANGE, 1000*msg.range_min)
        range_max = min(MAX_RANGE, 1000*msg.range_max)
        
        theta = angle_min
        obstList = [] 
        for dist in ranges:
            dist_mm = dist * 1000

            # On applique un masque pour supprimer les points qui ne sont pas dans les bornes indiquées (bornes de détection du LiDAR).
            if range_min<dist_mm<range_max:
                # Conversion de (d, theta) en (x_rel, y_rel)
                x_rel = dist_mm*math.cos(theta) + LIDAR_OFFSET_X
                y_rel = dist_mm*math.sin(theta) + LIDAR_OFFSET_Y
                obstacle = Point(x_rel, y_rel)
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
    node = LidarPositioningNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
