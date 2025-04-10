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

# pyright: reportMissingImports=false

"""
@file: disp_node.py
@status: OK

Fichier implementant le DisplacementNode.

Ce node intervient dans la gestion des deplacements sur commandes du
ActionNode de la strat. Il fait le lien entre la strat et la teensy.
Fichier executable du dossier.

NB: code [english], commentaires [french].
"""

#################################################################
#																#
# 							IMPORTS 							#
#																#
#################################################################
import sys
import traceback

import rclpy
from rclpy.node import Node

import numpy as np
from math import sqrt
import time

# import fonction du Pathfinder
from .pathfinder import PathFinder, PathNotFoundError

# import msgs
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16, Float32MultiArray, Int16MultiArray, String, Empty
from br_messages.msg import Position, Point, DisplacementOrder

# import utils
from .disp_utils import MAX_ASTAR_TIME, to_robot_coord, debug_print, INIT_ZONE

# import comms
from .disp_comm import DispCallbacks, SIMULATION, COM_STRAT

from config.qos import default_profile, latch_profile, br_position_topic_profile

#################################################################
#																#
# 						DISPLACEMENT NODE						#
#																#
#################################################################


class DisplacementNode(Node):

    """Classe du implementant le DisplacementNode."""

#######################################################################
# Initialisation 
#######################################################################
 
    def __init__(self):
        """Initialise le DisplacementNode."""
        super().__init__("DSP")
        self.get_logger().info("Initializing DSP Node ...")

        callbacks = DispCallbacks(self)
        self.color_sub = self.create_subscription(Int16, '/game/color', callbacks.setup_color, default_profile)
        self.init_pos_sub = self.create_subscription(Int16, '/game/init_pos', callbacks.setup_init_pos, default_profile)
        self.end_sub = self.create_subscription(Int16, '/game/end', callbacks.callback_end, default_profile)

        # Comm Teensy
        self.pub_teensy_go_to = self.create_publisher(DisplacementOrder, '/br/goTo', latch_profile)
        self.pub_teensy_stop = self.create_publisher(Empty, '/br/stop', latch_profile)
        self.pub_teensy_reset = self.create_publisher(Position, '/br/resetPosition', latch_profile)

        self.sub_teensy = self.create_subscription(Int16,  "/br/callbacks", callbacks.callback_teensy, default_profile) 
        self.sub_pos = self.create_subscription(Position,  "/br/currentPosition", callbacks.callback_position, br_position_topic_profile) 
   
        # Comm Lidar
        self.sub_lidar = self.create_subscription(Int16MultiArray, "/obstaclesInfo", callbacks.callback_lidar, default_profile)
        self.pub_speed = self.create_publisher(Int16, "/br/setSpeed", latch_profile)
        if SIMULATION:
            self.pub_obstacle = self.create_publisher(Int16MultiArray, "/simu/robotObstacle", latch_profile)

        # Comm Strat
        self.pub_strat = self.create_publisher(Int16, "/dsp/callback/next_move", latch_profile)
        self.sub_strat = self.create_subscription(Quaternion, "/dsp/order/next_move", callbacks.callback_strat, default_profile)

        # Obstacles
        self.sub_delete = self.create_subscription(String, "/removeObs", callbacks.callback_delete, default_profile)

        ############################
        #### Pour la Simulation ####
        ############################

        # Comm Simulation
        self.pub_grid = self.create_publisher(Float32MultiArray, "/simu/nodegrid", latch_profile)

        ## Variables liées au match

        self.init_pos = INIT_ZONE
        self.color = 0
        self.matchEnded = False

        ## Variables liées au fonctionnement de l'algorithme A* et de la création du chemin de points

        self.pathfinder = PathFinder(self.color, self.get_logger())
        self.max_astar_time = MAX_ASTAR_TIME

        ## Variable liées au déplacement du robot
        self.move = False                       # Le robot est en cours de deplacement
        self.forward = True                     # Le robot est en marche avant ? (False = marche arriere)
        self.current_pos = [0,0,0]                # La position actuelle du robot
        self.destination = None

        # ## Variables de mode de DEPLACEMENT
        self.avoid_mode = False                  # Le robot se deplace en mode evitement

        # ## Variables speciales
        self.reset_point = [0,0]                 # Point au alentour duquel il faut reset les marges d'arret de l'evitement
        self.is_reset_possible = False            # Variable décrivant si il faut reset les marges d'evitement ou non

        # ## Variables de gestion des obstacles / arrets
        self.wait_start = None

    def reset_position(self, x, y, theta):
        self.forward = True
        self.destination = None
        self.move = False

        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.pub_teensy_reset.publish(msg)

        self.pathfinder = PathFinder(self.color, self.get_logger()) 
        self.publish_grid()

    def publish_grid(self, grid=None):
        """Publish grid to the interfaceNode."""
        if SIMULATION:
            if grid is None: grid = self.pathfinder.get_grid()
            node_coords = []
            for n in range(len(grid)):
                node_coords.append(grid[n,0])
                node_coords.append(grid[n,1])
            
            msg = Float32MultiArray()
            msg.data = node_coords
            self.pub_grid.publish(msg)

    def set_destination(self, destination, forward=True, avoidMode=True):
        self.is_reset_possible = False
        self.destination = destination
        self.forward = forward
        self.avoid_mode = avoidMode

    def set_target_heading(self, orientation):
        self.forward = True
        self.destination = [orientation]

    def clear_destination(self):
        self.destination = None
        self.forward = True
        self.move = False

    def start_move(self):       
        if self.destination is not None:             
            self.wait_start = None
            if len(self.destination) == 1:
                self._start_rotation()
            else:
                self._start_go_to()
        else:
            self.get_logger().warning("start_move called before a destination is set")

    def _start_rotation(self):
        self.move = True
        msg = DisplacementOrder()
        msg.kind = DisplacementOrder.FINAL_ORIENTATION
        msg.theta = self.destination[0]
        self.pub_teensy_go_to.publish(msg)

    def _start_go_to(self):
        self.move = False
        try:
            begin_time = time.perf_counter()
            path = self.pathfinder.get_path(self.current_pos[:2], self.destination[:2])
            debug_print(self, 'c*', f"Time taken to build path : {time.perf_counter() - begin_time}")

            self.get_logger().info("Found path: \n"+str(path))

            self.move = True

            msg = DisplacementOrder()
            msg.path = [Point(x=point.x, y=point.y) for point in path]
            msg.kind = DisplacementOrder.ALLOW_CURVE | DisplacementOrder.FINAL_ORIENTATION
            if not self.forward:
                msg.kind |= DisplacementOrder.REVERSE
            msg.theta = self.destination[2] if len(self.destination) > 2 else 0.

            self.pub_teensy_go_to.publish(msg)

        except PathNotFoundError:       
            self.get_logger().warning("ERROR - Path not found")
            rsp = Int16()     
            rsp.data = COM_STRAT["path not found"]
            self.pub_strat.publish(rsp)

        except Exception as e:
            traceback.print_exception(e)
            self.get_logger().warning("ERROR - Exception: " + str(e))
            rsp = Int16()
            rsp.data = COM_STRAT["path not found"]
            self.pub_strat.publish(rsp)

    def stop_move(self):
        self.move = False
        self.pub_teensy_stop.publish(Empty())

    def set_avoid_reset_point(self):
        """Calcul du point de reset des marges d'evitement.
        
        Il s'agit du point à partir duquel on sera suffissament loin de
        l'obstacle jusqu'a la fin du trajet."""

        avoid_robot_pos = self.pathfinder.get_obstacle_robot_pos()
        if avoid_robot_pos is None:
            return # opponent position not known
        avoid_robot_pos = avoid_robot_pos[0]
        
        pos_list = [self.current_pos] + self.path
        i = len(pos_list) -1
        work = True
        while i > 0 and work: 
            #Parcours des droites de trajectoire de la fin vers le début
            if min(pos_list[i-1][0], pos_list[i][0])<avoid_robot_pos[0]<max(pos_list[i-1][0], pos_list[i][0]) or min(pos_list[i-1][1], pos_list[i][1])<avoid_robot_pos[1]<max(pos_list[i-1][1], pos_list[i][1]):
                if pos_list[i][0]-pos_list[i-1][0] !=0:
                    a = float(pos_list[i][1]-pos_list[i-1][1])/float(pos_list[i][0]-pos_list[i-1][0])
                    b=-1
                    c = pos_list[i-1][1]-a*pos_list[i-1][0]
                else:
                    a=1
                    b=0
                    c=-pos_list[i-1][0]
                #Calcul de la distance de la droite au centre de l'obstacle
                d = (a*avoid_robot_pos[0]+b*avoid_robot_pos[1]+c)/sqrt(a**2+b**2)
                if abs(d)<600:
                    work = False
            i-=1

        if work:    # On est toujours trop pres --> reset sur le point final
            self.reset_point = self.path[-1][:2]
        else:       # On calcul et setup le point de reset
            vect_normal = np.array([a,b])/np.linalg.norm([a,b])
            self.reset_point = np.array(avoid_robot_pos) - d*vect_normal
            
#################################################################
#																#
# 							MAIN PROG 							#
#																#
#################################################################

def main():
    rclpy.init(args=sys.argv)
    
    node = DisplacementNode()

    node.get_logger().info("Waiting for match to start")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

#################################################################
if __name__ == '__main__':
    main()