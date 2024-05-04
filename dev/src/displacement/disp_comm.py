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
MAX_RANGE = 600  

RADIUS_ROBOT_OBSTACLE = 280

STOP_RANGE_X = 600

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
    "stop":                 1,      # Le robot s'arrête
    "go":                   2,      # Le robot redémarre
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
    p_disp.match_ended = False 
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
        p_disp.match_ended = True

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

        begin_time = time.perf_counter()

        result = p_disp.build_path(p_disp.avoid_mode, p_disp.is_first_accurate)

        debug_print('c*', f"Time taken to build path : {time.perf_counter() - begin_time}")

        ## Si on a trouvé un chemin
        if result['success']:
            if p_disp.stop:
                log_info("New Possible Path")
                p_disp.stop = False
                pub_strat.publish(Int16(COM_STRAT["go"]))
            else :
                log_info("Found path: \n"+str(p_disp.path))
                # Affichage du path
                if len(p_disp.path) > 0:
                    publish_path(p_disp.path)
                if p_disp.avoid_mode:
                    #Calcul du point de reset des marges d'évitement
                    p_disp.set_avoid_reset_point()
            ## Sinon, erreur de la recherche de chemin
                p_disp.move = True 
                p_disp.next_point(False)
        else:       
            if p_disp.stop:
                log_warn("ERROR - Reason: Path Blocked")
                pub_strat.publish(Int16(COM_STRAT["stop blocked"]))
            elif result['message'] == "Dest Blocked":
                log_warn("ERROR - Reason: " + result['message'])
                pub_strat.publish(Int16(COM_STRAT["stop blocked"]))

            else:
                #NOTE no path found
                log_warn("ERROR - Reason: " + result['message'])
                # Retour de l'erreur a la strat
                pub_strat.publish(Int16(COM_STRAT["path not found"]))

                # Retry without opponents chaos
                if p_disp.avoid_mode:
                    result = p_disp.build_path(p_disp.avoid_mode, p_disp.is_first_accurate)
                    if result['success']:
                        log_info("Path found without chaos: [{}]".format(p_disp.path))
                        p_disp.set_avoid_reset_point()
                    else:
                        #TODO remove chaos stuff
                        log_info("Error without chaos: {}".format(result['message']))
                else:
                    log_info("Error: {}".format(result['message']))

    ## On envoie le premier point a la Teensy
    p_disp.move = True 
    p_disp.next_point(False)

def callback_lidar(msg):    
    """Fonction qui gere l'adaptation du robot aux obstacles (pas que lidar en fait...)."""

    if not ok_comm: return
    
    if (not p_disp.avoid_mode) or p_disp.matchEnded: 
        return

    ## Initialise parametre d'obstacle
    obstacle_info = np.zeros(5)     # infos sur l'obstacle
    obstacle_seen = False           # doit-on s'arreter ?
    obstacle_stop = False
    is_dest_blocked = False           # la destination est-elle accessible ?

    ## TRAITEMENT DE CHAQUE OBSTACLE
    nb_obstacles = (msg.layout.dim[0]).size
    if nb_obstacles == 0:
        # On est pas dans l'état bloqué et la pos ennemie est quelconque. On publie la vitesse nominale.
        p_disp.pathfinder.set_robot_to_avoid_pos([-1000, -1000], 0)    
        p_disp.blocked = False
        p_disp.stop = False
        pub_speed.publish(data=NOMINAL_SPEED)
    
    for i in range(nb_obstacles):
        # Si on est déjà à l'arrêt on ne rentre pas dans la boucle.
        if obstacle_stop: break
        for j in range(5):
            # Params de l'obstacle
            obstacle_info[j] = np.array(msg.data[5 * i + j + 1])
        # Info obstacles dans repere local du robot
        dist_obs = obstacle_info[2]

        #log_info("DIST OBS :" + str(dist_obs))
        x_loc_obs, y_loc_obs = to_robot_coord(p_disp.current_pos[0], p_disp.current_pos[1], p_disp.current_pos[2], obstacle_info)

####################################################################################################################################
####################################################################################################################################
        
        if p_disp.forward: 
            #-> FILTRER LES OBSTACLES AUX COORDONNEES EN DEHORS (SI CA MARCHE PAS DEJA)
            #-> NE PAS CHERCHER DE PATH INUTILEMENT SI LA DESTINATION EST DANS LA ZONE DE BLOCAGE DE L'OBSTACLE
            if msg.data[0] not in [0,1]:
                log_errs("Wrong msg from callback_obstacle.")
                continue
            #-> Si lidar : on regarde les adversaires devant !
            #-> On setup la vitesse suivant la pos locale du 
            #   robot adverse
            if msg.data[0] == 0:
                if dist_obs <= DIST_MIN: continue
                obstacle_seen = True
                if dist_obs <= STOP_RANGE_X:
                    if x_loc_obs >= abs(y_loc_obs) : 
                       # log_info("Adversary Detected In Front")
                        obstacle_stop = True
                        p_disp.pathfinder.set_robot_to_avoid_pos([obstacle_info[0], obstacle_info[1]], RADIUS_ROBOT_OBSTACLE)
                                                
        # Update de la vitesse??
        if not p_disp.blocked :
            if obstacle_stop:
                p_disp.blocked = True
                p_disp.stop = True 
                pub_teensy.publish(Quaternion(0, 0, 0, CMD_TEENSY["stop"]))         
                log_warn("Object Detected : Need To Wait")
                p_disp.pathfinder.set_robot_to_avoid_pos([obstacle_info[0], obstacle_info[1]], RADIUS_ROBOT_OBSTACLE)
                pub_strat.publish(Int16(COM_STRAT["stop blocked"]))

        if dist_obs > STOP_RANGE_X:
            p_disp.blocked = False

        speed_coeff = (dist_obs-MAX_RANGE)/(STOP_RANGE_X-MAX_RANGE)
        speed_coeff = min(0.5, max(0, speed_coeff))
        speed = NOMINAL_SPEED - int(speed_coeff*NOMINAL_SPEED)
        pub_speed.publish(data=speed) ## On prévient le BN qu'on a vu un truc et qu'il faut ralentirmaxSpeedLin


def callback_init_pos(msg):
    """Update la position de départ du robot."""
    if INIT_ZONE == 0:
        x, y, z = INIT_POS[0], INIT_POS[1], INIT_POS[2]
    elif INIT_ZONE == 1:
        x, y, z = INIT_POS2[0], INIT_POS2[1], INIT_POS2[2]
    elif INIT_ZONE == 2:
        x, y, z = INIT_POS3[0], INIT_POS3[1], INIT_POS2[2]
    
    if p_disp.color == 1:
        y = 3000 - y
        z = -z

    pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["set"]))
    p_disp.current_pos = [x, y, z]

    ## Init pathfinder with correct color
    p_disp.pathfinder = Pathfinder(p_disp.color) #TODO paramètre à supprimer, une seule grid commune pour le pathfinder
    publish_grid(p_disp.pathfinder.table_map.get_node_list())


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

def publish_path(path):
    """Publish path to the interfaceNode."""    
    if SIMULATION:
        path_coords = []
        for k in range (len(path)):
            path_coords.append(path[k][0])
            path_coords.append(path[k][1])
            if k == len(path)-1: path_coords.append(path[k][2])  # le cap final
        pub_path.publish(data = path_coords)  # liste des coordonnees successives
        log_info("## Simulation ## Path published : {}".format(path_coords))

def publish_grid(grid):
    """Publish grid to the interfaceNode."""
    if SIMULATION:
        node_coords = []
        for n in range(len(grid)):
            node_coords.append(grid[n].get_x())
            node_coords.append(grid[n].get_y())
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
