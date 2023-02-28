#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

from ast import literal_eval

# import pathfinder
from pathfinder.pathfinder import Pathfinder
# import msgs
from std_msgs.msg      import Int16, Int16MultiArray, Float32MultiArray
from geometry_msgs.msg import Quaternion, Pose2D
# import logs
from disp_utils import LOG_INFO, LOG_ERRS, LOG_WARN
from disp_utils import printablePos, toRobotCoord, patchFrameBR, dprint
from disp_utils import SIMULATION, READER, ROBOT_NAME

#################################################################################################
if os.environ['USER'] == 'pi':
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg 		# sur robot
else:
    from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

## CONSTANTES
BECAUSE_BIG_IS_BIG  = 100 if ROBOT_NAME=="GR" else 0
STOP_RANGE_STANDARD = 500 + BECAUSE_BIG_IS_BIG
STOP_RANGE_AVOIDING = 350 + BECAUSE_BIG_IS_BIG
RADIUS_ROBOT_OBSTACLE = 300
RESET_RANGE = 560  
STOP_RANGE_X_STAND = 650 + BECAUSE_BIG_IS_BIG
STOP_RANGE_X_AVOID = 500 + BECAUSE_BIG_IS_BIG
STOP_RANGE_Y_STAND = 250 + BECAUSE_BIG_IS_BIG
STOP_RANGE_Y_AVOID = 175 + BECAUSE_BIG_IS_BIG

COEFF_ANGLES = 0.57735026 # pi/6 | 30°

def init_comm(displacementNode):
    global p_dn   # create global variable pointer to DisplacementNode
    p_dn = displacementNode 

#######################################################################
# Dictionnaires des interfaces
#######################################################################

"""Dictionnaire des commandes envoyees a la Teensy."""
CMD_TEENSY = {
    "disp":                 0,      # Déplacement du robot vers un point
    "stop":                 1,      # Arrête le mouvement
    "accurate":             2,      # Déplacement précis du robot
    "recalage":             3       # Déplacement de type recalage (contre un bord du terrain typiquement)
}

"""Dictionnaire des commandes recues de la strat."""
CMD_STRAT = {
    "disp":                 0,      # Déplacement du robot vers un point
    "noAvoidance":          1,      # On désactive l'évitement pour ces déplacements
    "stop":                 2,      # Arrête le mouvement
    "accurate":             3,      # Déplacement précis du robot
    "recalage":             4       # Déplacement de type recalage (contre un bord du terrain typiquement)
}

"""Dictionnaire des callback renvoyees a la strat."""
COM_STRAT = {
    "asserv error":         -2,     # Erreur de l'asserv (difficile à gérer)
    "path not found":       -1,     # La recherche de chemin n'a pas aboutie
    "ok pos":               0,      # Le robot est arrivé au point demandé
    "stop":                 1,      # Le robot s'arrête
    "go":                   2,      # Le robot redémarre
    "stop blocked":         3       # On s'arrete car la destination est bloquee par l'adversaire
}


#######################################################################
# CALLBACK FUNCTIONS
#######################################################################

def callback_teensy(msg):
    """Traitement des msg recues de la teensy."""

    ## Problème asserv
    if msg.data == -1:
        LOG_INFO("ERROR - asserv.")
        pub_strat.publish(COM_STRAT["asserv error"])
        return

    ## On est arrivé a point 
    if msg.data == 0:
        LOG_INFO("Point reached. Go to next point.")
        p_dn.next_point(True)
        return
    
    LOG_INFO("Teensy cmd unknown. Callback msg.data = {}".format(msg.data))
    


def callback_strat(msg):
    """Traitement des commandes de la strat."""
    
    ## Reset des params
    # p_dn.accurateMode = False
    # p_dn.rotationMode = False
    # p_dn.avoidMode = False
    # p_dn.sameXMode = False
    # p_dn.avAccurateMode = False
    # p_dn.arAccurateMode = False

    # p_dn.finalTurn = False
    # p_dn.stop_obstacle_detection = False
    # p_dn.resume = False
    # p_dn.paused = False

    LOG_INFO("Order of displacement from AN: [{},{},{}] - method of displacement : [{}]".format(msg.x, msg.y, msg.z, msg.w))
    p_dn.path = []

    ## Commande d'arrêt
    if msg.w == CMD_STRAT["stop"]:
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY["stop"])) ## Les coordonnées ici importent peu car on demande de s'arrêter.
        p_dn.matchEnded = True

    ## Parametrage du mouvement selon la commande
    """ if msg.w == CMD_STRAT["front recal"]:
        p_dn.recalageMode = True
        p_dn.stop_obstacle_detection = True   #On ne detecte pas les obstacles lors des recalages
        p_dn.recalageParam = CMD_TEENSY["front recal"]
        p_dn.path = [[msg.x, msg.y, msg.z]] """

    if msg.w == CMD_STRAT["accurate"]:
        p_dn.path = [[msg.x, msg.y, msg.z]]
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY["accurate"]))

    elif msg.w == CMD_STRAT["recalage"]:
        p_dn.path = [[msg.x, msg.y, msg.z]]
        pub_teensy.publish(Quaternion(msg.x, msg.y, msg.z, CMD_TEENSY["recalage"]))

        

    # elif msg.w == CMD_STRAT["back recal"]:
    #     p_dn.recalageMode = True
    #     p_dn.stop_obstacle_detection = True
    #     p_dn.recalageParam = CMD_TEENSY["back recal"]
    #     p_dn.path = [[msg.x, msg.y, msg.z]]

    # elif msg.w == CMD_STRAT["front contact"]:
    #     p_dn.recalageMode = True
    #     p_dn.stop_obstacle_detection = True
    #     p_dn.recalageParam = CMD_TEENSY["contact_front"]
    #     p_dn.path = [[msg.x, msg.y, msg.z]]

    # elif msg.w == CMD_STRAT["back contact"]:
    #     p_dn.recalageMode = True
    #     p_dn.stop_obstacle_detection = True
    #     p_dn.recalageParam = CMD_TEENSY["contact_back"]
    #     p_dn.path = [[msg.x, msg.y, msg.z]]

    # elif msg.w == CMD_STRAT["accurate"]:
    #     p_dn.accurateMode = True
    #     p_dn.path = [[msg.x, msg.y, msg.z]]
    #     pub_speed.publish(data=[p_dn.speedLin, p_dn.speedRot])

    # elif msg.w == CMD_STRAT["accurate same x"]:
    #     p_dn.accurateMode = True
    #     p_dn.sameXMode = True
    #     p_dn.path = [[msg.x, msg.y, msg.z]]
    #     pub_speed.publish(data=[p_dn.speedLin, p_dn.speedRot])

    # elif msg.w == CMD_STRAT["accurate backward"]:
    #     p_dn.accurateMode = True
    #     p_dn.arAccurateMode = True            
    #     p_dn.path = [[msg.x, msg.y, msg.z]]
    #     pub_speed.publish(data=[0.4 * p_dn.maxSpeedLin, 0.6 * p_dn.maxSpeedRot])

    # elif msg.w == CMD_STRAT["accurate front"]:
    #     p_dn.accurateMode = True
    #     p_dn.avAccurateMode = True
    #     p_dn.path = [[msg.x, msg.y, msg.z]]
    #     pub_speed.publish(data=[0.8 * p_dn.maxSpeedLin, 1 * p_dn.maxSpeedRot])

    # elif msg.w == CMD_STRAT["rotation"] or msg.w == CMD_STRAT["slow rotation"]:
    #     p_dn.rotationMode = True
    #     p_dn.finalTurn = True
    #     p_dn.path = [[0, 0, msg.z]]

    #     if msg.w == CMD_STRAT["slow rotation"]: 
    #         pub_speed.publish(data=[p_dn.speedLin, p_dn.maxSpeedRot/12.0])
    #     else: 
    #         pub_speed.publish(data=[p_dn.speedLin, p_dn.speedRot])

    else:
        ## Setup de la vitesse
        p_dn.speedLin = p_dn.maxSpeedLin
        p_dn.speedRot = p_dn.maxSpeedRot
        pub_speed.publish(data=[p_dn.speedLin, p_dn.speedRot])

        # p_dn.maxAstarTime = 10

        if msg.w == CMD_STRAT["displacement"] or msg.w == CMD_STRAT["avoidance"]:
            ## Setup du Pathfinder
            dest_pos = [msg.x, msg.y, msg.z]
            curr_pos = p_dn.current_pos
            p_dn.pathfinder.setGoal(dest_pos)
            p_dn.pathfinder.setInit(curr_pos)

            ## - Deplacement classique
            if msg.w == CMD_STRAT["displacement"]: 
                # Affichage lisible
                aff_curr_pos = printablePos(curr_pos)
                aff_dest_pos = printablePos(dest_pos)
                LOG_INFO("Standard displacement :\n{} -> {}\n".format(aff_curr_pos, aff_dest_pos))

                result = p_dn.build_path(False, p_dn.isFirstAccurate, False)

            ## - Deplacement avec evitement
            else:   
                # Affichage lisible
                aff_curr_pos = printablePos(curr_pos)
                aff_dest_pos = printablePos(dest_pos)
                LOG_INFO("Avoiding displacement :\n{} -> {}\n".format(aff_curr_pos, aff_dest_pos))
                
                p_dn.avoidMode = True
                p_dn.isResetPossible = False

                result = p_dn.build_path(True, p_dn.isFirstAccurate, False)

            ## If pathfinding was a success 
            if result['success']:
                LOG_INFO("Found path: \n"+str(p_dn.path))
                # Affichage du path
                if len(p_dn.path) > 0:
                    publishPath(p_dn.path)
                if p_dn.avoidMode:
                    #Calcul du point de reset des marges d'évitement
                    p_dn.setAvoidResetPoint()
            ## Sinon, erreur de la recherche de chemin
            else:       
                LOG_WARN("ERROR - Reason: " + result['message'])
                # Retour de l'erreur a la strat
                pub_strat.publish(Int16(COM_STRAT["path not found"]))

                # Retry without opponents chaos
                if p_dn.avoidMode:
                    result = p_dn.build_path(True, p_dn.isFirstAccurate, True)
                    if result['success']:
                        LOG_INFO("Path found without chaos: [{}]".format(p_dn.path))
                        p_dn.setAvoidResetPoint()
                    else:
                        LOG_INFO("Error without chaos: {}".format(result['message']))
                else:
                    LOG_INFO("Error: {}".format(result['message']))

        elif msg.w == CMD_STRAT["straight line"]: 
            dest_pos = [msg.x, msg.y, msg.z]
            LOG_INFO("Found path: straight forward to" + str(dest_pos))
            p_dn.path = [dest_pos]

        elif msg.w == CMD_STRAT["relative pos"]:
            dest_pos = [msg.x, msg.y, msg.z] + p_dn.current_pos
            p_dn.path = [dest_pos]
        
    ## On envoie le premier point a la Teensy
    p_dn.move = True 
    p_dn.next_point(False)


def callback_lidar(msg):
    """Fonction qui gere l'adaptation du robot aux obstacles (pas que lidar en fait...)."""
    try:
        ## Initialise parametre d'obstacle
        obstacle_info = np.zeros(5)     # infos sur l'obstacle
        obstacle_seen = False           # doit-on s'arreter ?
        obstacle_stop = False
        isDestBlocked = False           # la destination est-elle accessible ?
        
        ####
        speedLin = p_dn.maxSpeedLin
        distMin = 5000
        stopRange = STOP_RANGE_STANDARD
        stopFrontX = STOP_RANGE_X_STAND
        stopFrontY = STOP_RANGE_Y_STAND
        # Distance à l'obstacle lidar pour laquelle on s'arrete en deplacement classique  # TODO : à paramétrer
        ####

        if not p_dn.move or p_dn.stop_detection_obstacle or p_dn.finish: return
        if p_dn.avoidMode: 
            stopRange = STOP_RANGE_AVOIDING
            stopFrontX = STOP_RANGE_X_AVOID
            stopFrontY = STOP_RANGE_Y_AVOID

        ## TRAITEMENT DE CHAQUE OBSTACLE
        nb_obstacles = (msg.layout.dim[0]).size
        for i in range(nb_obstacles):
            if obstacle_stop: break
            for j in range(5):
                # Params de l'obstacle
                obstacle_info[j] = np.array(msg.data[5 * i + j + 1])
            # Info obstacles dans repere local du robot
            distObs = obstacle_info[2]
            xLocObs, yLocObs = toRobotCoord(p_dn.current_pos[0], p_dn.current_pos[1], p_dn.current_pos[2], obstacle_info)

####################################################################################################################################
####################################################################################################################################
            # Si le robot tourne sur son axe (nb: on ne verif pas dans 
            # le cas du avoidMode ou resume car plutot bien alignes)
            if p_dn.turn and not p_dn.avoidMode and not p_dn.resume:
                if msg.data[0] not in [0,1]:
                    LOG_ERRS("Wrong msg from callback_obstacle.")
                    continue
                if msg.data[0] == 0:  # msg du lidar
                    if distObs < 0.9*stopRange:
                        obstacle_stop = True
            #-> Si le robot avance
            elif p_dn.forward: 
                #-> FILTRER LES OBSTACLES AUX COORDONNEES EN DEHORS (SI CA MARCHE PAS DEJA)
                #-> NE PAS CHERCHER DE PATH INUTILEMENT SI LA DESTINATION EST DANS LA ZONE DE BLOCAGE DE L'OBSTACLE
                if msg.data[0] not in [0,1]:
                    LOG_ERRS("Wrong msg from callback_obstacle.")
                    continue
                #-> Si lidar : on regarde les adversaires devant !
                #-> On setup la vitesse suivant la pos locale du 
                #   robot adverse
                if msg.data[0] == 0:
                    if distObs > distMin: continue
                    if distObs < stopRange:
                        obstacle_seen = True
                        if yLocObs == 0 or abs(yLocObs) < xLocObs*COEFF_ANGLES: 
                            obstacle_stop = True
                            # p_dn.pathfinder.setRobotToAvoidPos([obstacle_info[0], obstacle_info[1], RADIUS_ROBOT_OBSTACLE])
                    else:
                        if xLocObs < stopFrontX and abs(yLocObs) < stopFrontY:
                            obstacle_seen = True

            #-> Si le robot recule
            else:
                if msg.data[0] not in [0,1]: 
                    LOG_ERRS("Wrong msg from callback_obstacle.")
                    continue
                
                if msg.data[0] == 1: # sonar
                    dprint("sonar osbtacle, info obstacle=[{}] -- NOT HANDLED".format(obstacle_info))
                    # if abs(yLocObs) < 120:
                    #     obstacle_seen = True
                    #     p_dn.pathfinder.setRobotToAvoidPos([obstacle_info[0], obstacle_info[1]],RADIUS_ROBOT_OBSTACLE)  # TODO : à paramétrer    
                    # ecart sur le cote a partir duquel on considere l'obstacle  
                    # pour la simulation Docker, sinon un obstacle loin mais proche en xLoc est detecte
####################################################################################################################################
####################################################################################################################################
            
            if msg.data[0] == 0: # msg du lidar
                # Calcul de la distance à l'obstacle le plus proche
                if distObs < distMin: distMin = distObs
                
            # Reset des marges lors d'un évitement
            if p_dn.avoidMode and distMin > RESET_RANGE and p_dn.isResetPossible:  # TODO : à paramétrer LA DISTANCE A PARTIR DE LAQUELLE ON CONSIDERE QUE CE N'EST PLUS UN EVITEMENT ####
                p_dn.avoidMode = False

            # Update de la vitesse??
            if obstacle_seen: speedLin = 0.5*p_dn.maxSpeedLin

        if p_dn.speedLin != speedLin: #update if different!
            p_dn.speedLin=speedLin
            pub_speed.publish(data=[speedLin, p_dn.maxSpeedRot])
        p_dn.handle_obstacle(obstacle_seen, obstacle_stop, isDestBlocked)
    except:
        pass    

def callback_color(msg):
    """Update la couleur du robot."""
    try:
        ## Init starting position 
        homePos = list(literal_eval(READER.get("Robot", "start_pos")))
        awayPos = [homePos[0], 3000-homePos[1], -homePos[2]]
        
        if msg.data == 0: # HOME
            p_dn.color_txt = "Yellow"
            p_dn.color_int = 0
            #pub_teensy.publish(Quaternion(homePos[0], homePos[1], homePos[2], 3))
            
            # TODO - remove patch
            x, y, c = patchFrameBR(homePos[0], homePos[1], homePos[2])
            pub_teensy.publish(Quaternion(x,y,c,3))
        else:
            p_dn.color_txt = "Purple"
            p_dn.color_int = 1
            #pub_teensy.publish(Quaternion(awayPos[0], awayPos[1], awayPos[2], 3))

            # TODO - remove patch
            x, y, c = patchFrameBR(awayPos[0], awayPos[1], awayPos[2])
            pub_teensy.publish(Quaternion(x,y,c,3))

        ## Init pathfinder with correct color
        p_dn.pathfinder = Pathfinder(p_dn.color_int)
        publishGrid(p_dn.pathfinder.tableMap.getNodeList())
    except:
        pass


def callback_position(msg):
    """Update la position actuelle du robot."""
    try:
        # # Traitement des infos
        # p_dn.current_pos = np.array([msg.x, msg.y, msg.theta])

        # TODO - remove patch
        x,y,c = patchFrameBR(msg.x, msg.y, msg.theta)
        p_dn.current_pos = np.array([x,y,c])

        # On reset les marges si assez proche du point de reset
        if p_dn.avoidMode and np.linalg.norm(p_dn.current_pos[:2]-p_dn.resetPoint) < 20:
            p_dn.isResetPossible = True    
    except:
        pass

def callback_speed(msg):
    """Update la vitesse ordonnée par la strat """
    p_dn.maxSpeedLin = msg[0]
    p_dn.maxSpeedRot = msg[1]
    return

def publishPath(path):
    """Publish path to the interfaceNode."""    
    if SIMULATION:
        pathCoords = []
        for k in range (len(path)):
            pathCoords.append(path[k][0])
            pathCoords.append(path[k][1])
            if k == len(path)-1: pathCoords.append(path[k][2])  # le cap final
        pub_path.publish(data = pathCoords)  # liste des coordonnees successives
        LOG_INFO("## Simulation ## Path published : {}".format(pathCoords))

def publishGrid(grid):
    """Publish grid to the interfaceNode."""
    if SIMULATION:
        nodeCoords = []
        for n in range(len(grid)):
            nodeCoords.append(grid[n].getX())
            nodeCoords.append(grid[n].getY())
        pub_grid.publish(data=nodeCoords)
        LOG_INFO("## Simulation ## Grid published to interface.")

#######################################################################
# PUBLISHERS & SUBSCRIBERS
#######################################################################

# Comm Teensy
pub_teensy = rospy.Publisher('/nextPositionTeensy', Quaternion, queue_size=10, latch=True)
sub_teensy = rospy.Subscriber("/okPosition", Int16, callback_teensy) 

# Comm Lidar
sub_lidar = rospy.Subscriber("/obstaclesInfo", Int16MultiArray, callback_lidar)

# Comm Strat
pub_strat = rospy.Publisher("/okPositionPF", Int16, queue_size=10, latch=False)
sub_strat = rospy.Subscriber("/nextDisplacement", Quaternion, callback_strat)
sub_color = rospy.Subscriber("/color", Int16, callback_color)

# Comm Position
sub_pos = rospy.Subscriber("/current_position", Pose2D, callback_position)

# Publication parametres de jeu & gains
sub_speed = rospy.Subscriber("/speedStrat", Float32MultiArray, callback_speed)
pub_speed = rospy.Publisher("/speedTeensy", Float32MultiArray, queue_size=10, latch=False)

############################
#### Pour la Simulation ####
############################

# Comm Pathfinder
pub_path = rospy.Publisher("/simu/current_path", Float32MultiArray, queue_size=10, latch=False)
# Comm Simulation
pub_grid = rospy.Publisher("/simu/nodegrid", Float32MultiArray, queue_size=10, latch=False)
