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
import rospy
import numpy as np
from math import sin
import time

from ast import literal_eval

# import pathfinder
from pathfinder.pathfinder import Pathfinder
# import msgs
from std_msgs.msg      import Int16, Int16MultiArray, Float32MultiArray, String
from geometry_msgs.msg import Quaternion, Pose2D
# import logs
from disp_utils import *

#################################################################################################
from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

## CONSTANTES
NOMINAL_SPEED = 80
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

COEFF_ANGLES = 0.57735026 # pi/6 | 30°

SIMULATION = True

def init_comm(displacementNode):
    global p_disp   # create global variable pointer to DisplacementNode
    p_disp = displacementNode 
    global ok_comm
    ok_comm = True 

#######################################################################
# Dictionnaires des interfaces
#######################################################################

'''Dictionnaire des commandes envoyees a la Teensy'''

CMD_TEENSY = {
    "disp":                 1,      # Déplacement du robot vers un point (transitoire, pas le dernier)
    "dispFinal":            0,      # Déplacement du robot vers un point final de traj
    "stop":                 2,      # Arrête le mouvement
    "accurate":             5,      # Déplacement précis du robot vers l'avant
    "recalage":             6,      # Déplacement de type recalage arrière (bumper qu'à l'arrière(contre un bord du terrain typiquement))
    "rotation":             9,      # Cas où on fait une rotation simple
    "set":                  3,      # Fixe la position de départ
    "wii":                  4,       # Cas d'utilisation de la manette wii
    "marcheArr":            8
}


'''Dictionnaire des commandes recues de la strat'''
CMD_STRAT = {
    "standard":     0,        # Déplacement du robot vers un point
    "noAvoidance":  1,     # On désactive l'évitement pour ces déplacements
    "stop":         2,            # Arrête le mouvement
    "accurate":     3,        # Déplacement précis du robot
    "recalage":     4,        # Déplacement de type recalage (contre un bord du terrain typiquement)
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
    "okPos": 1,
    "okTurn": 2,
    "marcheArrOK": 3
}


#######################################################################
# CALLBACK FUNCTIONS
#######################################################################

global ok_comm
ok_comm = False 

def setup_color(msg):
    """
    Callback function from topic /sm/color.
    """
    if msg.data not in [0,1]:
        log_errs(f"Wrong value of color given ({msg.data})...")
        return
    else: 
        p_disp.color = msg.data
        log_info("Received color : {}".format(COLOR[p_disp.color]))
        callback_init_pos(msg)

def callback_teensy(msg):
    """Traitement des msg recues de la teensy."""

    ## Problème asserv
    if msg.data == CB_TEENSY["errorAsserv"]:
        log_info("ERROR - asserv.")
        pub_strat.publish(COM_STRAT["asserv error"])
        return

    ## On est arrivé a point (okPos)
    if msg.data == CB_TEENSY["okPos"]:
        if p_disp.final_move:
            log_info("Arrived to position")
            p_disp.turn = True
            p_disp.final_turn = True
            p_disp.avoid_mode = False
        else:
            log_info("Go to path next point.")
            p_disp.next_point(True)
        return
    
    ## okTurn de la part de la Teensy
    if msg.data == CB_TEENSY["okTurn"]:
        p_disp.turn = False
        p_disp.resume = False

        if p_disp.final_turn:
            p_disp.final_turn = False
            log_info("Final orientation done.")
            p_disp.next_point(True)
        return
    
    if msg.data == CB_TEENSY["marcheArrOK"]:
        """ if p_disp.blocked:
            p_disp.blocked = False
            pub_strat.publish(Int16(COM_STRAT["ok pos"])) """
        p_disp.marche_arr_dest = None
        p_disp.move = False
        log_info("Reverse Gear done.")
        return
    
    log_info("Teensy cmd unknown. Callback msg.data = {}".format(msg.data))
    


def callback_strat(msg):
    """Traitement des commandes de la strat."""
    
    ## Reset des params
    p_disp.accurate = False
    p_disp.rotation = False
    p_disp.recalage = False 
    p_disp.move = False
    p_disp.avoid_mode = False

    # p_disp.finalTurn = False
    # p_disp.stop_obstacle_detection = False
    # p_disp.resume = False
    # p_disp.paused = False

    log_info("Order of displacement from AN: [{},{},{}] - method of displacement : [{}]".format(msg.x, msg.y, msg.z, msg.w))
    #log_info("Robot State : " + str(p_disp.blocked))
    p_disp.path = []

    ## Commande d'arrêt
    if msg.w == CMD_STRAT["stop"]:
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY["stop"])) ## Les coordonnées ici importent peu car on demande de s'arrêter.
        p_disp.move = False
        self.final_move = False

    elif msg.w == CMD_STRAT["accurate"]:
        p_disp.path = [[msg.x, msg.y, msg.z]]
        p_disp.accurate = True
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY["accurate"]))

    elif msg.w == CMD_STRAT["recalage"]:
        p_disp.path = [[msg.x, msg.y, msg.z]]
        p_disp.recalage = True
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY["recalage"]))
    
    elif msg.w == CMD_STRAT["rotation"]:
        p_disp.rotation = True
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY['rotation']))

    elif msg.w == CMD_STRAT["marcheArr"]:
        p_disp.marche_arr_dest = [msg.x, msg.y, msg.z]
        p_disp.move = True
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY['marcheArr']))
    
    elif msg.w == CMD_STRAT["noAvoidance"]:
        p_disp.avoid_mode = False
        p_disp.is_reset_possible = False
        p_disp.move = True
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY['dispFinal']))

    elif msg.w == CMD_STRAT["standard"] or msg.w == CMD_STRAT["noAvoidance"]:
        ## Setup de la vitesse

        dest_pos = [msg.x, msg.y, msg.z]
        curr_pos = p_disp.current_pos

        ## - Déplacement standard
        if msg.w == CMD_STRAT["standard"] :
            log_info("Standard displacement :\n{} -> {}\n".format(printable_pos(curr_pos), printable_pos(dest_pos)))
            p_disp.avoid_mode = True
            p_disp.is_reset_possible = False
            p_disp.move = True

            ## Setup du Pathfinder
            p_disp.max_astar_time = MAX_ASTAR_TIME
            p_disp.pathfinder.set_goal(dest_pos)
            p_disp.pathfinder.set_init(curr_pos)            

        ## - Deplacement sans evitement
        else:   
            log_info("Displacement without avoidance :\n{} -> {}\n".format(printable_pos(curr_pos), printable_pos(dest_pos)))
            p_disp.avoid_mode = False
            p_disp.is_reset_possible = False
            p_disp.move = True

            ## Setup du Pathfinder
            p_disp.max_astar_time = MAX_ASTAR_TIME
            p_disp.pathfinder.set_goal(dest_pos)
            p_disp.pathfinder.set_init(curr_pos)

        p_disp.move_forward()

def callback_lidar(msg):    
    """Fonction qui gere l'adaptation du robot aux obstacles (pas que lidar en fait...)."""

    def set_opponent_pos(pos, radius):
        p_disp.pathfinder.set_robot_to_avoid_pos([pos[0], pos[1]] if pos is not None else None, radius) # TODO center
        if SIMULATION:
            msg2 = Int16MultiArray()
            msg2.data = [int(pos[0]) if pos is not None else -10000, int(pos[1]) if pos is not None else -10000, int(radius)]
            pub_obstacle.publish(msg2)

    if not ok_comm: return
    
    if (not p_disp.avoid_mode) or p_disp.matchEnded: 
        return

    if msg.data[0] not in [0,1]:
        log_errs("Wrong msg from callback_obstacle.")
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

        x_loc_obs, y_loc_obs = to_robot_coord(p_disp.current_pos[0], p_disp.current_pos[1], p_disp.current_pos[2], obstacle_info)

        if msg.data[0] == 0 and x_loc_obs > 0 and (closest_obs_ahead is None or dist_obs < closest_obs_ahead[0]):
            closest_obs_ahead = (dist_obs, x_loc_obs, y_loc_obs)
            set_opponent_pos([obstacle_info[0], obstacle_info[1]], RADIUS_ROBOT_OBSTACLE+STOP_RANGE_X+MARGIN) # TODO center
          
        if msg.data[0] == 1 and x_loc_obs < 0 and (closest_obs_behind is None or dist_obs < closest_obs_behind[0]):
            closest_obs_behind = (dist_obs, x_loc_obs, y_loc_obs)

    
    speed = NOMINAL_SPEED    
    if p_disp.forward: 
        #-> FILTRER LES OBSTACLES AUX COORDONNEES EN DEHORS (SI CA MARCHE PAS DEJA)
        #-> NE PAS CHERCHER DE PATH INUTILEMENT SI LA DESTINATION EST DANS LA ZONE DE BLOCAGE DE L'OBSTACLE
        
        #-> Si lidar : on regarde les adversaires devant !
        #-> On setup la vitesse suivant la pos locale du 
        #   robot adverse
        
        if closest_obs_ahead is not None:
            dist_obs, x_loc_obs, y_loc_obs = closest_obs_ahead

            # We trust the path finder to avoid the obstacle
            # Bypassing may require going a little closer to the obstacle before moving away                i   
            if dist_obs <= STOP_RANGE_X or (x_loc_obs < STOP_RANGE_X and abs(y_loc_obs) < STOP_RANGE_Y):
                log_warn("Object Detected Too Close : Interrupting move")
                pub_teensy.publish(Quaternion(0, 0, 0, CMD_TEENSY["stop"]))
                pub_strat.publish(Int16(COM_STRAT["stop blocked"]))
            
            elif not p_disp.bypassing and (dist_obs <= BYPASS_RANGE_X or (x_loc_obs < BYPASS_RANGE_X and abs(y_loc_obs) < BYPASS_RANGE_Y)):
                pub_teensy.publish(Quaternion(0, 0, 0, CMD_TEENSY["stop"]))
                if p_disp.wait_start is None:
                    log_warn("Object Detected : Need to wait")
                    p_disp.wait_start = time.perf_counter()
                    pub_teensy.publish(Quaternion(0, 0, 0, CMD_TEENSY["stop"]))
                elif time.perf_counter() - p_disp.wait_start >= STAND_BEFORE_BYPASS:                            
                    log_warn("Object Won't Move Out Of The Way  : Initiating bypass")                              
                    p_disp.move_forward() # Recompute path with updated opponent pos
                else:
                    log_warn(str(p_disp.wait_start) + "--" + str(time.perf_counter() - p_disp.wait_start))
            
            elif p_disp.wait_start is not None:
                log_info("Object Has Cleared The Way : Resuming displacement")     
                p_disp.move_forward()
    
            if dist_obs < SLOWDOWN_RANGE:
                speed_coeff = 1 - (dist_obs-SLOWDOWN_RANGE)/(STOP_RANGE_X-SLOWDOWN_RANGE)
                speed_coeff = max(0.5, min(1, speed_coeff))
                speed *= speed_coeff

        else:
            set_opponent_pos(None, 0) 
            if not p_disp.bypassing and p_disp.wait_start is not None:
                log_info("Object Has Cleared The Way : Resuming displacement")     
                p_disp.move_forward()

    else:
        # No avoiding strategy when reversing, because only straight, short-distance reverses are used
        if closest_obs_behind is not None:
            dist_obs, x_loc_obs, y_loc_obs = closest_obs_behind
            if dist_obs < STOP_RANGE_X and p_disp.move:
                log_warn("Object Detected In The Back : Need to wait")
                pub_teensy.publish(Quaternion(0, 0, 0, CMD_TEENSY["stop"]))
        
        if not p_disp.move and p_disp.marche_arr_dest is not None:
            log_info("Object Has Cleared The Way : Resuming reverse")
            p_disp.move = True
            pub_teensy.publish(Quaternion(*p_disp.marche_arr_dest, CMD_TEENSY['marcheArr']))

    if p_disp.bypassing and p_disp.is_reset_possible:
        p_disp.bypassing = False
        p_disp.wait_start = None
        set_opponent_pos(None, 0)

    pub_speed.publish(data=int(speed)) ## On prévient le cas échéant le BN qu'on a vu un truc et qu'il faut ralentir


def callback_init_pos(msg):
    """Update la position de départ du robot."""
    if INIT_ZONE == 0:
        x, y, z = INIT_POS[0], INIT_POS[1], INIT_POS[2]
    elif INIT_ZONE == 1:
        x, y, z = INIT_POS2[0], INIT_POS2[1], INIT_POS2[2]
    elif INIT_ZONE == 2:
        x, y, z = INIT_POS3[0], INIT_POS3[1], INIT_POS3[2]
    
    if p_disp.color == 1:
        y = 3000 - y
        z = -z

    pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["set"]))
    p_disp.current_pos = [x, y, z]

    ## Init pathfinder with correct color
    p_disp.pathfinder = Pathfinder(p_disp.color) 
    publish_grid(p_disp.pathfinder.table_map.get_grid().reshape(-1,2))


def callback_position(msg):
    """Update la position actuelle du robot."""

    if not ok_comm: return 
    p_disp.current_pos = [msg.x, msg.y, msg.theta]
    
    # On reset les marges si assez proche du point de reset
    if p_disp.avoid_mode and np.linalg.norm([p_disp.current_pos[0] - p_disp.reset_point[0], p_disp.current_pos[1] - p_disp.reset_point[1]]) < 20:
        p_disp.is_reset_possible = True    

def callback_end(msg):
    if not ok_comm: return
    if msg.data == 1:
        p_disp.matchEnded = True

def callback_delete(msg):
    if not ok_comm: return
    p_disp.pathfinder.remove_obstacle(msg.data)

def publish_grid(grid):
    """Publish grid to the interfaceNode."""
    if SIMULATION:
        node_coords = []
        for n in range(len(grid)):
            node_coords.append(grid[n,0])
            node_coords.append(grid[n,1])
        pub_grid.publish(data=node_coords)
        log_info("## Simulation ## Grid published to interface.")

#######################################################################
# PUBLISHERS & SUBSCRIBERS
#######################################################################

color_sub = rospy.Subscriber('/game/color', Int16, setup_color)
end_sub = rospy.Subscriber('/game/end', Int16, callback_end)

# Comm Teensy
pub_teensy = rospy.Publisher('/nextPositionTeensy', Quaternion, queue_size=10, latch=True)
sub_teensy = rospy.Subscriber("/okPosition", Int16, callback_teensy) 

# Comm Lidar
sub_lidar = rospy.Subscriber("/obstaclesInfo", Int16MultiArray, callback_lidar)
pub_speed = rospy.Publisher("/teensy/obstacle_seen", Int16, queue_size=10, latch=True)
if SIMULATION:
    pub_obstacle = rospy.Publisher("/simu/robotObstacle", Int16MultiArray, queue_size=10, latch=True)

# Comm Strat
pub_strat = rospy.Publisher("/dsp/callback/next_move", Int16, queue_size=10, latch=False)
sub_strat = rospy.Subscriber("/dsp/order/next_move", Quaternion, callback_strat)

# Comm Position
sub_pos = rospy.Subscriber("/current_position", Pose2D, callback_position)

# Obstacles
sub_delete = rospy.Subscriber("/deleteObs", String, callback_delete)


""" # Publication parametres de jeu & gains
sub_speed = rospy.Subscriber("/param/speedStrat", Float32MultiArray, callback_speed)
pub_speed = rospy.Publisher("/param/speedTeensy", Float32MultiArray, queue_size=10, latch=False) """

############################
#### Pour la Simulation ####
############################

# Comm Pathfinder
pub_path = rospy.Publisher("/simu/current_path", Float32MultiArray, queue_size=10, latch=False)
# Comm Simulation
pub_grid = rospy.Publisher("/simu/nodegrid", Float32MultiArray, queue_size=10, latch=False)
