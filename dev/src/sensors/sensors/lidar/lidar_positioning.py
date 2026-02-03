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
        self.mark_msg = SensorObstacleList()

        self.mark_1 = Point(x=MARK_1_X, y=MARK_1_Y)
        self.mark_2 = Point(x=MARK_2_X, y=MARK_2_Y)
        self.mark_3 = Point(x=MARK_3_X, y=MARK_3_Y)

        self.dist_12 = self.mark_1.euclidean_distance(self.mark_2)
        self.dist_13 = self.mark_1.euclidean_distance(self.mark_3)
        self.dist_23 = self.mark_2.euclidean_distance(self.mark_3)

        self.mark_1_pos_rel = Point(x=MARK_1_X, y=MARK_1_Y)
        self.mark_2_pos_rel = Point(x=MARK_2_X, y=MARK_2_Y)
        self.mark_3_pos_rel = Point(x=MARK_3_X, y=MARK_3_Y)

        # initialisation des publishers
        self.pub_obstacles = self.create_publisher(SensorObstacleList, "/sensors/obstaclesLidar", default_profile)
        self.pub_position = self.create_publisher(Position, "/br/currentPosition", br_position_topic_profile)
        self.pub_marks = self.create_publisher(SensorObstacleList, "/sensors/marksLidar", default_profile)
        
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
        self.robot_pos = self.CALL_NIELS_FUNCTION_HERE(marks) 
        #Input : une liste de longueur 3 de la position relative des marks détectées (que se passe-t-il si il n'y en a que 2 ?) 
        #Output : la position absolue du robot (x, y theta)

        #On passe les coordonnées dans le repère absolu
        clusters_abs = []
        marks_abs = []

        cos = math.cos(self.robot_pos.theta)
        sin = math.sin(self.robot_pos.theta)

        for cluster in clusters:
            x_abs, y_abs = make_absolute(self.robot_pos, cluster, cos=cos, sin=sin)
            clusters_abs.append(Point(x_abs, y_abs))

        for mark in marks:
            x_abs, y_abs = make_absolute(self.robot_pos, mark, cos=cos, sin=sin)
            marks_abs.append(Point(x_abs, y_abs))

        #On filtre les points hors de la table
        length = len(clusters_abs)
        i = 0

        while (i < length):
            point = clusters_abs[i]
            if not((0 < point.x < TABLE_W) and (0 < point.y < TABLE_H)):
                clusters_abs[i], clusters_abs[length-1] = clusters_abs[length-1], clusters_abs[i]
                clusters_abs.pop()
                i -= 1
                length -= 1
            i += 1

        #On publie la position du robot, des repères et des obstacles
        self.obstacle_msg.obstacles.clear()
        self.mark_msg.obtacles.clear()

        for cluster in clusters_abs:
            self.obstacle_msg.obstacles.append(SensorObstacle(cluster.x, cluster.y))

        for mark in marks_abs:
            self.mark_msg.obstacles.append(SensorObstacle(mark.x, mark.y))

        self.pub_obstacles.publish(self.obstacle_msg)
        self.pub_marks.publish(self.mark_msg)
        self.pub_position.publish(self.robot_pos)

    def _get_marks(self, clusters):
        """ 
        Parmi les clusters, on cherche les trois points qui correspondent à la position des repères
            Si un seul set convient, on le retourne
	        Si deux sets ou plus conviennent, on prend celui le plus proche des repères précédents
	        Si aucun set ne convient, on test avec seulement deux points. De même que précédemment si plusieurs sets conviennent
	        Si aucun set de deux points ne convient non plus, on log une erreur et on attend le prochain scan 
        """
        
        valid_marks = []

        #On cherche tous les sets de trois points
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

        #Si un seul convient, on le retourne   
        if (len(valid_marks == 1)):
            return valid_marks[0]
        
        #Si deux sets ou plus conviennent, on renvoie celui le plus proche du set à l'itération précédente
        elif (len(valid_marks) > 1):
            return self._get_dist_set_min(valid_marks)






        return valid_marks

    def _get_dist_set_min(self, sets):
        i_min = 0
        min_dist = sys.maxsize

        mark_1 = self.mark_1_pos_rel
        mark_2 = self.mark_2_pos_rel
        mark_3 = self.mark_3_pos_rel

        for j in range(len(sets)):
                marks = sets[j]
                dist_old_marks = min(marks[0].euclidean_distance(mark_1) + marks[1].euclidean_distance(mark_2) + marks[2].euclidean_distance(mark_3), \
                                     marks[0].euclidean_distance(mark_1) + marks[2].euclidean_distance(mark_2) + marks[1].euclidean_distance(mark_3), \
                                     marks[1].euclidean_distance(mark_1) + marks[0].euclidean_distance(mark_2) + marks[2].euclidean_distance(mark_3), \
                                     marks[1].euclidean_distance(mark_1) + marks[2].euclidean_distance(mark_2) + marks[0].euclidean_distance(mark_3), \
                                     marks[2].euclidean_distance(mark_1) + marks[0].euclidean_distance(mark_2) + marks[1].euclidean_distance(mark_3), \
                                     marks[2].euclidean_distance(mark_1) + marks[1].euclidean_distance(mark_2) + marks[0].euclidean_distance(mark_3)  )
                if (dist_old_marks < min_dist):
                    i_min = j
                    min_dist = dist_old_marks
        return sets[i_min]

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
                x_rel = dist_mm*math.cos(theta - LIDAR_ANGLE*math.pi/180) + LIDAR_OFFSET_X
                y_rel = dist_mm*math.sin(theta - LIDAR_ANGLE*math.pi/180) + LIDAR_OFFSET_Y
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
    

def _compute_robot_position_from_xbyb(self, b1_true_pos, b2_true_pos, b1_robot_pos, b2_robot_pos):
    """
    Détermine la position et l'orientation du robot à partir des positions de deux balises
    dans le repère monde et dans le repère robot.

    Args:
        b1_true_pos (tuple): (x, y) de la balise 1 dans le repère monde.
        b2_true_pos (tuple): (x, y) de la balise 2 dans le repère monde.
        b1_robot_pos (tuple):  (x, y) de la balise 1 dans le repère robot.
        b2_robot_pos (tuple): ((x, y) de la balise 2 dans le repère robot.

    Returns:
        tuple: (position_robot_x, position_robot_y, orientation_theta)
    """

    # Unpack
    d1, a1 = b1_distanceangle
    d2, a2 = b2_distanceangle
    b1_local = np.array(list(b1_robot_pos))
    b2_local = np.array(list(b2_robot_pos))

    # Vecteur global balise 1 -> balise 2 dans le repère monde
    b1b2_global = np.array(b2_true_pos) - np.array(b1_true_pos)
    # Vecteur local balise 1 -> balise 2 dans le repère Robot
    b1b2_local = b2_local - b1_local

    # Système linéaire A * u = B pour trouver cos(theta) et sin(theta)
    # On cherche u = [cos, sin].
    x12r, y12r = b1b2_local 
    M_theta = np.array([
        [x12r, -y12r],
        [y12r,  x12r]
    ])
    u = np.linalg.pinv(M_theta) @ b1b2_global
    cos_theta_raw, sin_theta_raw = u

    # Si les distances mesurées ne sont pas parfaites, cos^2 + sin^2 != 1.
    norm = np.hypot(cos_theta_raw, sin_theta_raw)
    #print(norm)
    cos_theta = cos_theta_raw / norm
    sin_theta = sin_theta_raw / norm

    # Récupération de l'orientation du robot et l'expédie entre 0 et 2 pi
    theta_final = np.arctan2(sin_theta, cos_theta)%(2*np.pi)

    # Matrice de rotation reconstruite
    R = np.array([
        [cos_theta, -sin_theta],
        [sin_theta,  cos_theta]
    ])

    pos_robot = np.array(b1_true_pos) - (R @ b1_local)

    return pos_robot[0], pos_robot[1], theta_final









        
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
