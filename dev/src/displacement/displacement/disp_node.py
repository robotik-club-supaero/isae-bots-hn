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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import numpy as np
from math import sqrt
import time

# import fonction du Pathfinder
from .pathfinder.pathfinder import Pathfinder
from .pathfinder.exceptions import PathNotFoundError, TimeOutError, DestBlockedError

# import msgs
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import Int16, Float32MultiArray, Int16MultiArray, String

# import utils
from .disp_utils import MAX_ASTAR_TIME, to_robot_coord, debug_print, INIT_ZONE

# import comms
from .disp_comm import DispCallbacks, SIMULATION, COM_STRAT, CMD_TEENSY

from strat.strat_utils import create_quaternion

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

        latch_profile =  QoSProfile(
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Transient Local durability
        )
        callbacks = DispCallbacks(self)
        self.color_sub = self.create_subscription(Int16, '/game/color', callbacks.setup_color, 10)
        self.init_pos_sub = self.create_subscription(Int16, '/game/init_pos', callbacks.setup_init_pos, 10)
        self.end_sub = self.create_subscription(Int16, '/game/end', callbacks.callback_end, 10)

        # Comm Teensy
        self.pub_teensy = self.create_publisher(Quaternion, '/nextPositionTeensy', latch_profile)
        self.sub_teensy = self.create_subscription(Int16,  "/okPosition", callbacks.callback_teensy, 10) 

        # Comm Lidar
        self.sub_lidar = self.create_subscription(Int16MultiArray, "/obstaclesInfo", callbacks.callback_lidar, 10)
        self.pub_speed = self.create_publisher(Int16, "/teensy/obstacle_seen", latch_profile)
        if SIMULATION:
            self.pub_obstacle = self.create_publisher(Int16MultiArray, "/simu/robotObstacle", latch_profile)

        # Comm Strat
        self.pub_strat = self.create_publisher(Int16, "/dsp/callback/next_move", latch_profile)
        self.sub_strat = self.create_subscription(Quaternion, "/dsp/order/next_move", callbacks.callback_strat, 10)

        # Comm Position
        self.sub_pos = self.create_subscription(Pose2D, "/current_position", callbacks.callback_position, 10)

        # Obstacles
        self.sub_delete = self.create_subscription(String, "/removeObs", callbacks.callback_delete, 10)

        ############################
        #### Pour la Simulation ####
        ############################

        # Comm Pathfinder
        self.pub_path = self.create_publisher(Float32MultiArray, "/simu/current_path", latch_profile)
        # Comm Simulation
        self.pub_grid = self.create_publisher(Float32MultiArray, "/simu/nodegrid", latch_profile)


        ## Variables liées au match

        self.init_pos = INIT_ZONE
        self.color = 0
        self.matchEnded = False

        ## Variables liées au fonctionnement de l'algorithme A* et de la création du chemin de points

        self.path = []
        self.pathfinder = Pathfinder(self.color, self.get_logger())
        self.max_astar_time = MAX_ASTAR_TIME

        ## Variable liées au déplacement du robot
        self.move = False                       # Le robot est en cours de deplacement
        self.final_move = False                  # Le robot se dirige vers le point final
        self.turn = False
        self.final_turn = False 
        self.resume = False 
        self.paused = False                     # Le robot est arrete a cause d'un obstacle
        self.forward = True                     # Le robot est en marche avant ? (False = marche arriere)
        self.current_pos = [0,0,0]                # La position actuelle du robot

        # ## Variables de mode de DEPLACEMENT
        self.accurate = False             # Le robot se deplace precisement / lentement vers l'avant
        self.recalage = False               # Le robot se recale contre un mur / ou vient seulement en contact
        self.avoid_mode = False                  # Le robot se deplace en mode evitement
        self.rotation = False 

        # ## Variables speciales
        self.reset_point = [0,0]                 # Point au alentour duquel il faut reset les marges d'arret de l'evitement
        self.is_reset_possible = False            # Variable décrivant si il faut reset les marges d'evitement ou non
        self.is_first_accurate = False            # Variable permettant de savoir si le robot est dans un obstacle lors d'un evitement (savoir si on recule ou non)

        # ## Variables de gestion des obstacles / arrets
        self.marche_arr_dest = None
        self.bypassing = False
        self.wait_start = None

    def publish_path(self, path):
        """Publish path to the interfaceNode."""    
        if SIMULATION:
            path_coords = []
            for k in range (len(path)):
                path_coords.append(path[k][0])
                path_coords.append(path[k][1])
                if k == len(path)-1: path_coords.append(path[k][2])  # le cap final
            
            path_msg = Float32MultiArray()
            path_msg.data = path_coords
            self.pub_path.publish(path_msg)  # liste des coordonnees successives
            self.get_logger().info("## Simulation ## Path published : {}".format(path_coords))

#######################################################################
# Fonctions de construction de path
#######################################################################

    def build_path(self, isInAvoidMode, isFirstAccurate):
        """Fonction appelee quand on cherche chemin et le pathfinder setup.
        
        Retourne un chemin de la forme:
            [[x_1, y_1, theta_1], ..., [x_dest, y_dest, theta_dest]]

        avec (x_1, y_1), ... (x_n, y_n) fournis par le pathfinder, et les 
        angles calcules tels que:
            
        Y  
        ^
        |      (x2, y2)
        |    +
        |
        |    ^
        |    | theta1
        |    +
        |      (x1,y1)
        |  ^
        | /
        |/ theta0    
        +----------> X
        (x0,y0)

        le resultat de la fonction est un dictionnaire qui contient:
            - msg: un message d'erreur le cas echeant
            - success: si un chemin a ete trouve
            - chemin_calcule: le chemin calcule."""

        # Update temps max de l'Astar
        self.pathfinder.set_max_astar_time(self.max_astar_time)

        # Instanciation de resultisInAvoidMode
        result = {'message':"", 'success':False}

        # On essaie d'obtenir un chemin
        try:
            path = self.pathfinder.get_path(isInAvoidMode, isFirstAccurate)
            if not len(path):
                self.get_logger().error("Error - Empty path found")
                result['message'] = "Empty path found" 
                result['success'] = False
                return result

            self.path = path
            result['message'] = "Path found" 
            result['success'] = True

        except PathNotFoundError:   
            result['message'] = "Path not found"
            result['success'] = False
            return result
        
        except TimeOutError:
            result['message'] = "Time out"
            result['success'] = False
            return result

        except DestBlockedError:
            result['message'] = "Dest Blocked"
            result['success'] = False
            return result

        return result

    def move_forward(self):

        if self.bypassing:
            self.get_logger().info("Obstacle found ahead. Computing new path to bypass it...")

        self.wait_start = None
        self.final_move = False     

        begin_time = time.perf_counter()

        result = self.build_path(self.avoid_mode, self.is_first_accurate)

        debug_print(self, 'c*', f"Time taken to build path : {time.perf_counter() - begin_time}")

        ## Si on a trouvé un chemin
        if result['success']:
            self.get_logger().info("Found path: \n"+str(self.path))
            # Affichage du path
            if len(self.path) > 0:
                self.publish_path(self.path)

            self.move = True 
            self.next_point(False)

        ## Sinon, erreur de la recherche de chemin
        else:
            rsp = Int16()
            if self.bypassing:
                self.get_logger().warning("ERROR - Reason: Cannot bypass obstacle")
                rsp.data = COM_STRAT["stop blocked"]
            elif result['message'] == "Dest Blocked":
                self.get_logger().warning("ERROR - Reason: " + result['message'])
                rsp.data = COM_STRAT["stop blocked"]
            else:
                #NOTE no path found
                self.get_logger().warning("ERROR - Reason: " + result['message'])
                # Retour de l'erreur a la strat
                rsp.data = COM_STRAT["path not found"]
                
            self.pub_strat.publish(rsp)

    def next_point(self, just_arrived):
        """Envoie une commande du prochain point a la Teensy.
        
        - Soit on vient d'arriver au point (just_arrived=True) --> on supp 
        le premier point du path.
        - Soit on etait a un obstacle ou on vient de repartir --> on garde 
        le premier point du path (dest temporaire).
        
        Si il n'y a plus de point, on vient d'arriver, notif a la strat.
        Et gestion des differents modes de deplacements si param avant."""

        # Si on vient d'arriver, on retire le premier point si possible
        if just_arrived:
            if len(self.path) > 0:
                self.path = self.path[1:]
        
        # S'il existe un point 
        # S'il existe un point intermediaire
        if len(self.path) >= 2:
            x = self.path[0][0]
            y = self.path[0][1]
            self.get_logger().info("\nDisplacement Pass By ({}, {})".format(x,y))

            """ # TODO - remove patch
            x,y,_ = patch_frame_br(x,y,0,self.color) """

            # Si on est dans un obstacle
            if self.is_first_accurate:
                xLoc, _ = to_robot_coord(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.path[0])
                if xLoc > 0: # Marche avant necessaire
                    self.forward = True
                    self.pub_teensy.publish(create_quaternion(x,y,self.current_pos[2],CMD_TEENSY["accurate"]))
                else:
                    self.forward = False
                    self.pub_teensy.publish(create_quaternion(x,y,self.current_pos[2],CMD_TEENSY["accurate"]))
            else:
                self.forward = True
                self.pub_teensy.publish(create_quaternion(x,y,0,CMD_TEENSY["disp"]))

            # Init params du mouvement
            self.turn = True
            self.final_move = False
            return

        # Sinon, s'il reste un point c'est le dernier
        if len(self.path) == 1:
            x, y, c = self.path[0]
            self.turn = True
            self.final_move = True

            """ # TODO - remove patch 
            x,y,c = patch_frame_br(x,y,c) """

            ## Gestion differents types de deplacement
            if self.accurate:
                xRob, yRob, cRob = self.current_pos

                self.get_logger().info("\nDisplacement request ({}, {}, {}) accurate".format(x, y, c))
                
                xLoc, _ = to_robot_coord(xRob, yRob, cRob, [x,y,c])
                if xLoc >= 0:
                    self.pub_teensy.publish(create_quaternion(x, y, c, CMD_TEENSY["accurate"]))
                    self.forward = True
                else:
                    self.pub_teensy.publish(create_quaternion(x, y, c, CMD_TEENSY["accurate"]))
                    self.forward = False
                return

            if self.rotation:
                self.get_logger().info("\nDisplacement request ({}, {}, {}) rotation.".format(x, y, c))
                #print("\n\n\n\nspeedRot = {}\n\n\n\n".format(self.speedRot))
                self.pub_teensy.publish(create_quaternion(x, y, c, CMD_TEENSY["rotation"]))
                return

            if self.recalage:
                self.get_logger().info("\nDisplacement request ({}, {}, {}) recalage.".format(x, y, c))
                self.final_move = False     #Pas d'orientation finale en fin de recalage 
                self.pub_teensy.publish(create_quaternion(x, y, c, CMD_TEENSY["recalage"]))
                return
        
            ## Point final
            self.get_logger().info("\nDisplacement request ({}, {}, {}) standard.".format(x, y, c))
            self.forward = True
            self.pub_teensy.publish(create_quaternion(x, y, c, CMD_TEENSY["dispFinal"]))
            
        # Sinon, on on a fini (ou bien a nulle part ou aller, pcq obstacle
        # detecte sans qu'on bouge...)
        if len(self.path) == 0 and just_arrived:
            self.get_logger().info("Arrived at destination!")
            # Reset des params
            self.move = False
            self.turn = False
            self.final_turn = False
            self.final_move = False

            self.avoid_mode = False
            self.recalage = False
            self.accurate = False
            self.rotation = False 

            self.bypassing = False
            self.wait_start = None

            # Publication a la strat
            rsp = Int16()
            rsp.data = COM_STRAT["ok pos"]
            self.pub_strat.publish(rsp)


    def set_avoid_reset_point(self):
        """Calcul du point de reset des marges d'evitement.
        
        Il s'agit du point à partir duquel on sera suffissament loin de
        l'obstacle jusqu'a la fin du trajet."""

        avoid_robot_pos = self.pathfinder.get_robot_to_avoid_pos()
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
    
    time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

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