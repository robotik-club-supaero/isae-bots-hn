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
@file: disp_comm.py
@status: OK

Fichier du DisplacementNode contenant les outils de communication.
"""

#################################################################
#																#
# 							IMPORTS 							#
#																#
#################################################################

import os
import numpy as np
from math import sin
import time

# import pathfinder
from .pathfinder import PathFinder
# import msgs
from std_msgs.msg      import Int16, Int16MultiArray, Float32MultiArray, String
from geometry_msgs.msg import Quaternion
# import logs
from .disp_utils import *

#################################################################################################
from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

## CONSTANTES
NOMINAL_SPEED = 100
DIST_MIN = 50
RADIUS_ROBOT_OBSTACLE = 150

STAND_BEFORE_BYPASS = 2 # seconds
# As the opponent is expected to be moving, it should not stay on the path
# It may be more efficient to wait for it to free the way instead of initiating bypass

SLOWDOWN_RANGE = 700

BYPASS_RANGE_X = 300
BYPASS_RANGE_Y = 150

STOP_RANGE_X = 100
STOP_RANGE_Y = 50

SIMULATION = True

#######################################################################
# Dictionnaires des interfaces
#######################################################################

'''Dictionnaire des commandes envoyees a la Teensy'''

'''Dictionnaire des commandes recues de la strat'''
CMD_STRAT = {
    "standard":     0,        # Déplacement du robot vers un point
    "noAvoidance":  1,     # On désactive l'évitement pour ces déplacements # DEPRECATED
    "stop":         2,            # Arrête le mouvement
    "accurate":     3,        # Déplacement précis du robot # DEPRECATED
    "recalage":     4,        # Déplacement de type recalage (contre un bord du terrain typiquement) # DEPRECATED
    "rotation":     5,         # On ne demande qu'une rotation (Peut servir)
    "marcheArr":    8
}

'''Dictionnaire des callback renvoyees a la strat'''
COM_STRAT = {
    "asserv error":         -2,     # Erreur de l'asserv (difficile à gérer)
    "path not found":       -1,     # La recherche de chemin n'a pas aboutie
    "ok pos":               0,      # Le robot est arrivé au point demandé
    "stop":                 1,      # Le robot s'arrête # DEPRECATED, no longer published TODO cleanup
    "go":                   2,      # Le robot redémarre # DEPRECATED, no longer published TODO cleanup
    "stop blocked":         3,       # On s'arrete car la destination est bloquee par l'adversaire
    "possible path":        4
}

'''Dictionnaire des callbacks reçus de la teensy'''
CB_TEENSY = {
    "errorAsserv": 0,
    "okPos": 1, # DEPRECATED
    "okTurn": 2, # DEPRECATED
    "marcheArrOK": 3, # DEPRECATED
    "okOrder": 7,
}


#######################################################################
# CALLBACK FUNCTIONS
#######################################################################

# NOTE: these functions are moved here to limit length of file "disp_node.py"
class DispCallbacks:
    def __init__(self, node):
        self._node = node

    def setup_color(self, msg):
        """
        Callback function from topic /game/color.
        """
        if msg.data not in [0,1]:
            self._node.get_logger().error(f"Wrong value of color given ({msg.data})...")
            return
        else: 
            self._node.color = msg.data
            self._node.get_logger().info("Received color : {}".format(COLOR[self._node.color]))
            self.callback_init_pos()

    def setup_init_pos(self, msg):
        """
        Callback function from topic /game/init_pos
        """
        if msg.data not in [0,1,2]:
            self._node.get_logger().error(f"Wrong value of init pos given ({msg.data})...")
            return
        else: 
            self._node.init_pos = msg.data
            self._node.get_logger().info("Received init pos : {}".format(msg.data))
            self.callback_init_pos()

    def callback_teensy(self, msg):
        """Traitement des msg recues de la teensy."""

        ## Problème asserv
        if msg.data == CB_TEENSY["errorAsserv"]:
            self._node.get_logger().info("ERROR - asserv.")

            msg = Int16()
            msg.data = COM_STRAT["asserv error"]
            self._node.pub_strat.publish(msg)
            return

        if msg.data == CB_TEENSY["okOrder"]:
            self._node.clear_destination()

            rsp = Int16()
            rsp.data = COM_STRAT["ok pos"]
            self._node.pub_strat.publish(rsp)
            return

        if msg.data == CB_TEENSY["okPos"]:
            return
        
        if msg.data == CB_TEENSY["okTurn"]:
            return
        
        if msg.data == CB_TEENSY["marcheArrOK"]:
            return
        
        self._node.get_logger().info("Teensy cmd unknown. Callback msg.data = {}".format(msg.data))
        
    def callback_strat(self, msg):
        """Traitement des commandes de la strat."""
        ## Commande d'arrêt
        if msg.w == CMD_STRAT["stop"]:
            self._node.stop_move()
            self._node.clear_destination()

        elif msg.w == CMD_STRAT["rotation"]:
            self._node.set_target_heading(msg.z)
            self._node.start_move()
        
        elif msg.w == CMD_STRAT["standard"] or msg.w == CMD_STRAT["noAvoidance"] or msg.w == CMD_STRAT["marcheArr"]: 
            curr_pos = self._node.current_pos
            dest_pos = [msg.x, msg.y, msg.z]
            ## - Déplacement standard
            if msg.w == CMD_STRAT["standard"] :
                self._node.get_logger().info("Standard displacement :\n{} -> {}\n".format(printable_pos(curr_pos), printable_pos(dest_pos)))
            elif msg.w == CMD_STRAT["marcheArr"]:
                self._node.get_logger().info("Reverse displacement :\n{} -> {}\n".format(printable_pos(curr_pos), printable_pos(dest_pos)))
            ## - Deplacement sans evitement
            else:   
                self._node.get_logger().info("Displacement without avoidance :\n{} -> {}\n".format(printable_pos(curr_pos), printable_pos(dest_pos)))

            self._node.set_destination(dest_pos, msg.w != CMD_STRAT["marcheArr"], msg.w != CMD_STRAT["noAvoidance"])   
            self._node.start_move()

    def callback_lidar(self, msg):    
        """Fonction qui gere l'adaptation du robot aux obstacles (pas que lidar en fait...)."""

        def set_opponent_pos(pos, radius):
            self._node.pathfinder.set_obstacle_robot_pos([pos[0], pos[1]] if pos is not None else None, radius) # TODO center
            if SIMULATION:
                msg2 = Int16MultiArray()
                msg2.data = [int(pos[0]) if pos is not None else -10000, int(pos[1]) if pos is not None else -10000, int(radius)]
                self._node.pub_obstacle.publish(msg2)
                self._node.publish_grid()

        
        if (not self._node.avoid_mode) or self._node.matchEnded: 
            return

        if msg.data[0] not in [0,1]:
            self._node.get_logger().error("Wrong msg from callback_obstacle.")
            return

        ## Initialise parametre d'obstacle
        obstacle_info = np.zeros(5)     # infos sur l'obstacle
        closest_obs_ahead = None # dist, x_loc, y_loc
        closest_obs_behind = None

        ## TRAITEMENT DE CHAQUE OBSTACLE
        nb_obstacles = (msg.layout.dim[0]).size

        for i in range(nb_obstacles):
            for j in range(5):
                # Params de l'obstacle
                obstacle_info[j] = np.array(msg.data[5 * i + j + 1])
            # Info obstacles dans repere local du robot
            dist_obs = obstacle_info[2]
            dist_obs -= RADIUS_ROBOT_OBSTACLE # FIXME
            if dist_obs <= DIST_MIN: continue

            x_loc_obs, y_loc_obs = to_robot_coord(self._node.current_pos[0], self._node.current_pos[1], self._node.current_pos[2], obstacle_info)

            if msg.data[0] == 0 and x_loc_obs > 0 and (closest_obs_ahead is None or dist_obs < closest_obs_ahead[0]):
                closest_obs_ahead = (dist_obs, x_loc_obs, y_loc_obs)
                set_opponent_pos([obstacle_info[0], obstacle_info[1]], RADIUS_ROBOT_OBSTACLE+STOP_RANGE_X+MARGIN) # TODO center
            
            if msg.data[0] == 1 and x_loc_obs < 0 and (closest_obs_behind is None or dist_obs < closest_obs_behind[0]):
                closest_obs_behind = (dist_obs, x_loc_obs, y_loc_obs)

        
        speed = NOMINAL_SPEED    
        if self._node.forward: 
            #-> FILTRER LES OBSTACLES AUX COORDONNEES EN DEHORS (SI CA MARCHE PAS DEJA)
            #-> NE PAS CHERCHER DE PATH INUTILEMENT SI LA DESTINATION EST DANS LA ZONE DE BLOCAGE DE L'OBSTACLE
            
            #-> Si lidar : on regarde les adversaires devant !
            #-> On setup la vitesse suivant la pos locale du 
            #   robot adverse
            
            if closest_obs_ahead is not None:
                dist_obs, x_loc_obs, y_loc_obs = closest_obs_ahead

                # We trust the path finder to avoid the obstacle
                # Bypassing may require going a little closer to the obstacle before moving away
                if dist_obs <= STOP_RANGE_X or (x_loc_obs < STOP_RANGE_X and abs(y_loc_obs) < STOP_RANGE_Y):
                    self._node.get_logger().warning("Object Detected Too Close : Interrupting move")
                    self._node.stop_move()

                    rsp = Int16()
                    rsp.data = COM_STRAT["stop blocked"]
                    self._node.pub_strat.publish(rsp)
                
                elif dist_obs <= BYPASS_RANGE_X or (x_loc_obs < BYPASS_RANGE_X and abs(y_loc_obs) < BYPASS_RANGE_Y):
                    self._node.stop_move()
                    if self._node.wait_start is None:
                        self._node.get_logger().warning("Object Detected : Need to wait")
                        self._node.wait_start = time.perf_counter()
                        self._node.stop_move()
                    elif time.perf_counter() - self._node.wait_start >= STAND_BEFORE_BYPASS:                            
                        self._node.get_logger().warning("Object Won't Move Out Of The Way  : Initiating bypass")                              
                        self._node.start_move() # Recompute path with updated opponent pos
                
                elif self._node.wait_start is not None:
                    self._node.get_logger().info("Object Has Cleared The Way : Resuming displacement")     
                    self._node.start_move()
        
                if dist_obs < SLOWDOWN_RANGE:
                    speed_coeff = 1 - (dist_obs-SLOWDOWN_RANGE)/(STOP_RANGE_X-SLOWDOWN_RANGE)
                    speed_coeff = max(0.5, min(1, speed_coeff))
                    speed *= speed_coeff

            else:
                set_opponent_pos(None, 0) 
                if self._node.wait_start is not None:
                    self._node.get_logger().info("Object Has Cleared The Way : Resuming displacement")     
                    self._node.start_move()

        else:
            if closest_obs_behind is not None:
                dist_obs, x_loc_obs, y_loc_obs = closest_obs_behind
                if dist_obs < STOP_RANGE_X and self._node.move:
                    self._node.get_logger().warning("Object Detected In The Back : Need to wait")
                    self._node.stop_move()
            
            if not self._node.move:
                self._node.get_logger().info("Object Has Cleared The Way : Resuming reverse")
                self._node.start_move()

        if self._node.is_reset_possible:
            self._node.wait_start = None
            set_opponent_pos(None, 0)

        speedmsg = Int16()
        speedmsg.data = int(speed)
        self._node.pub_speed.publish(speedmsg) ## On prévient le cas échéant le BN qu'on a vu un truc et qu'il faut ralentir


    def callback_init_pos(self):
        """Update la position de départ du robot."""
        x, y, z = INIT_POS[self._node.init_pos]
        if self._node.color == 1:
            y = 3000 - y
            z = -z

        self._node.reset_position(x, y, z)

    def callback_position(self, msg):
        """Update la position actuelle du robot."""
 
        self._node.current_pos = [msg.x, msg.y, msg.theta]
        
        # On reset les marges si assez proche du point de reset
        if self._node.avoid_mode and np.linalg.norm([self._node.current_pos[0] - self._node.reset_point[0], self._node.current_pos[1] - self._node.reset_point[1]]) < 20:
            self._node.is_reset_possible = True    

    def callback_end(self, msg):
        if msg.data == 1:
            self._node.matchEnded = True

    def callback_delete(self, msg):
        self._node.pathfinder.remove_obstacle(msg.data)
        self._node.publish_grid()
