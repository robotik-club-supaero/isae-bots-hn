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
import os
import rclpy
from rclpy.node import Node

import numpy as np
from br_messages.msg import Position
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

class SensorsNode(Node):
    DISTANCEMIN = 100
    INTERVALLE = 1
    LIMITEBASSE = 10
    obstaclesLidar = []
    obstaclesSonars = []
    countLidar = 1
    countSonars = 1
    x_robot=0
    y_robot=0
    cap = 0

    # pour le test#
    def lineAngle(x0, y0, x1, y1):

        if x1 == x0:
            if y1 == y0:
                self.get_logger().error("ERREUR : memes coordonnes pour les deux points de la line")
            theta = np.sign(y1 - y0)*np.pi/2

        elif y1 == y0:
            theta = (1 - np.sign(x1 - x0))*np.pi/2

        else:
            theta = np.arctan((y1 - y0)/(x1 - x0))

        if x0>x1:  # cas ou l'arctan vaut en fait theta +- pi
            if y0<y1:
                theta = theta + np.pi
            else:
                theta = theta - np.pi
        return theta


    def update_position(self,msg):
        """Fonction de callback de position."""
        
        self.x_robot = msg.x
        self.y_robot = msg.y
        self.c_robot = msg.theta


	#fonction callback sur retour d'obstacles
    #ébauche de traitement des données LIDAR et SONAR pour déterminer des vitesses/directions. Ce code n'est pas efficace
    def update_obstacles(self, msg):
        #le premier indice de la liste vaut 0 si ce sont des obstacles LIDAR, 1 si SONAR
        if msg.data[0] == 0:
            count = self.countLidar
            obstacles = self.obstaclesLidar
            self.countLidar = (self.countLidar + 1) % self.INTERVALLE
        elif msg.data[0] == 1:
            count = self.countSonars
            obstacles = self.obstaclesSonars
            self.countSonars = (self.countSonars + 1) % self.INTERVALLE
        if count==0:
            infoList =[]
            newObstacles = []
            for i in range((msg.layout.dim[0]).size):
                newObstacles.append([msg.data[2*i +1], msg.data[2*i + 2]])
            for newObstacle in newObstacles:
                dx = 0
                dy = 0
                #on va déterminer les directions et vitesses (partie à améliorer)
                if (len(obstacles) != 0):                              #pour chaque obstacle on va chercher l'obstacle précédent le plus proche
                    nearestObstacle = obstacles[0]                                 #si ce dernier est assez proche, on suppose que c'est le même obstacle qui s'est déplacé
                    for obstacle in obstacles:
                        if np.linalg.norm(np.array(obstacle) - np.array(newObstacle)) < np.linalg.norm(np.array(nearestObstacle) -np.array(newObstacle)):
                            nearestObstacle = obstacle 
                    if np.linalg.norm( np.array(nearestObstacle) - np.array(newObstacle)) < self.DISTANCEMIN :
                        dx = newObstacle[0] - nearestObstacle[0]
                        dy = newObstacle[1] - nearestObstacle[1]
                        if np.linalg.norm([dx,dy]) < self.LIMITEBASSE:
                            dx = 0
                            dy = 0
                info = [newObstacle[0], newObstacle[1], ((int)(np.linalg.norm([self.x_robot - newObstacle[0], self.y_robot- newObstacle[1]]))), dx, dy]
                infoList.append(info)
                if msg.data[0] == 0:
                    self.obstaclesLidar = newObstacles
                elif msg.data[0] == 1:
                    self.obstaclesSonars = newObstacles

            #on renvoie toutes les caractéristiques calculées sur un autre topic

            newmsg = Int16MultiArray()

            data = [msg.data[0]]
            for infoObstacle in infoList:
                for info in infoObstacle:
                    data.append(info)

            newmsg.data = data

            layout = MultiArrayLayout()
            layout.data_offset = 1

            dimensions = []
            dim1 = MultiArrayDimension()
            dim1.label = "obstacle_nb"
            dim1.size = len(infoList)
            dim1.stride = 5* len(infoList)

            dim2 = MultiArrayDimension()
            dim2.label = "info"
            dim2.size = 5
            dim2.stride = 5

            dimensions.append(dim1)
            dimensions.append(dim2)

            layout.dim = dimensions

            newmsg.layout = layout

            self.pub_obstaclesInfo.publish(newmsg)

         
	
    def __init__(self):
        
        super().__init__('SensorsNode')
        self.get_logger().info("Initializing SensorsNode ...")

        self.pub_obstaclesInfo=self.create_publisher(Int16MultiArray, "/obstaclesInfo", 10)

        # initialisation des suscribers
        qos_profile = 10
        self.subLidar=self.create_subscription(Int16MultiArray, "/sensors/obstaclesLidar", self.update_obstacles, qos_profile)
        self.subSonars = self.create_subscription(Int16MultiArray, "/sensors/obstaclesSonar", self.update_obstacles, qos_profile)
        self.sub_rospy=self.create_subscription(Position, "/br/currentPosition", self.update_position, qos_profile)

    
def main():
    rclpy.init(args=sys.argv)

    node = SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()