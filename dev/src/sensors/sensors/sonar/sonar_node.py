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

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np

from geometry_msgs.msg import Point
from br_messages.msg import Position

from message.msg import SensorObstacleList, SensorObstacle
from message_utils.geometry import make_absolute
from config import SonarConfig
from config.qos import default_profile, br_position_topic_profile

from .sonar_config import *

#################################################################
#                                                               #
#                         SONAR node                            #
#                                                               #
#################################################################

class Sonar:
    def __init__(self, x, y, direction):
        self.x = x
        self.y = y
        self.dir = direction
        self.cos = math.cos(direction)
        self.sin = math.sin(direction)

    def get_coords(self, dist):
        x = self.x + dist * self.cos
        y = self.y + dist * self.sin

        return x, y

class SonarNode(Node):
    """
    ROS node SONAR node for sonar obstacles detection.
    """

    def __init__(self):
        super().__init__("SON")
        self.get_logger().info("Initializing SonarNode ...")
        
        # Get sonars from config file of the robot
        config = SonarConfig()
        
        self.sonars_lst = [Sonar(*sonar) for sonar in config.available_sonars]     #on enregistre chaque sonar (sa position et son cap) dans cette liste

        self.pos_robot = Position()

        self.obs_pub = self.create_publisher(SensorObstacleList, "/sensors/obstaclesSonar", default_profile)
        self.pos_sub = self.create_subscription(Position, "/br/currentPosition", self.recv_position, br_position_topic_profile)
        self.son_sub = self.create_subscription(Point, "/ultrasonicDistances", self.recv_obstacle, default_profile)  # can change topic name ?
        # FIXME: using type `Point` for `/ultrasonicDistances` only works if there are two sonars

        self.obs_msg = SensorObstacleList()
    
    def recv_position(self, msg):
        """
        Feedback on current position /br/currentPosition topic.
        """
        self.pos_robot = msg

    def recv_obstacle(self, msg):
        """
        Feedback on obstacle info from sonars
        """

        self.obs_msg.obstacles.clear()
        dst_lst = [msg.x, msg.y] # FIXME this only works for two sonars

        for dst, sonar in zip(dst_lst, self.sonars_lst):
            if distance < DIST_MARGIN:
                x, y = sonar.get_coords(dst)

                if DROP_OFF_LIMITS:
                    x_abs, y_abs = make_absolute(robot_pos, Point(x=x,y=y))
                    if not AREA_MARGIN < y_obs < 3000-AREA_MARGIN:
                        continue
                    if not AREA_MARGIN < x_obs < 3000-AREA_MARGIN:
                        continue
                
                self.obs_msg.obstacles.push(Obstacle(x=x,y=y,dist=dst))

        self.obs_pub.publish(info)


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)
    
    node = SonarNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
