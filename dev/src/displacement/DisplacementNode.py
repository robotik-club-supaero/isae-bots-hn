#!/usr/bin/env python
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
@file: DisplacementNode.py
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
import rospy
import numpy as np
from math import sqrt
from time import time

print("version : ",sys.version)

# import fonction du Pathfinder
from pathfinder.pathfinder import Pathfinder
from pathfinder.exceptions import PathNotFoundError, TimeOutError

# import msgs
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16, Float32MultiArray

# import utils
from disp_utils import LOG_INFO, LOG_ERRS, dprint, toRobotCoord, patchFrameBR

# import comms
from disp_comm import COM_STRAT, CMD_TEENSY, pub_strat, pub_teensy, pub_speed

#################################################################
#																#
# 						DISPLACEMENT NODE						#
#																#
#################################################################

class DisplacementNode:

    """Classe du implementant le DisplacementNode."""

#######################################################################
# Initialisation 
#######################################################################
 
    def __init__(self):
        """Initialise le DisplacementNode."""

        LOG_INFO("Initializing Displacement Node.")

        ## Variables liées au match

        self.color = 0
        self.config = 0
        self.matchEnded = False

        ## Variables liées à la vitesse de déplacement du robot

        self.maxSpeedLin = 0
        self.maxSpeedRot = 0

        ## Variables liées au fonctionnement de l'algorithme A* et de la création du chemin de points

        self.path = []
        self.pathfinder = Pathfinder(self.color)
        self.maxAstarTime = 5

        ## Variable liées au déplacement du robot
        self.move = False                       # Le robot est en cours de deplacement
        self.finalMove = False                  # Le robot se dirige vers le point final
        self.forward = True                     # Le robot est en marche avant ? (False = marche arriere)
        self.current_pos = [0,0]                # La position actuelle du robot

        # ## Variables de mode de DEPLACEMENT
        self.accurateMode = False             # Le robot se deplace precisement / lentement vers l'avant
        self.recalageMode = False               # Le robot se recale contre un mur / ou vient seulement en contact
        self.avoidMode = False                  # Le robot se deplace en mode evitement

        # ## Variables speciales
        self.resetPoint = [0,0]                 # Point au alentour duquel il faut reset les marges d'arret de l'evitement
        self.isResetPossible = False            # Variable décrivant si il faut reset les marges d'evitement ou non
        self.isFirstAccurate = False            # Variable permettant de savoir si le robot est dans un obstacle lors d'un evitement (savoir si on recule ou non)

        # ## Variables de gestion des obstacles / arrets
        self.blocked = False                     # Le robot est arrete a cause d'un obstacle

#######################################################################
# Fonctions de construction de path
#######################################################################

    def build_path(self, isInAvoidMode, isFirstAccurate, isSecondAttempt):
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
        self.pathfinder.setMaxAstarTime(self.maxAstarTime)

        # Instanciation de resultisInAvoidMode
        result = {'message':"", 'success':False, 'built path':[]}

        # On essaie d'obtenir un chemin
        try:
            result['built path'] = self.pathfinder.getPath(isInAvoidMode, isFirstAccurate, isSecondAttempt)
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

        # On recupere le chemin obtenu
        if not len(result['built path']):
            LOG_ERRS("Error - Empty path found")
            result['message'] = "Empty path found" 
            result['success'] = False
            return result
        
        self.path = result['built path']
        return result


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
        if len(self.path):
            x = self.path[0][0]
            y = self.path[0][1]
            z = self.path[0][2]
            x, y, z = patchFrameBR(x, y, z)

            if self.recalageMode:
                LOG_INFO("\nDisplacement request ({}, {}, {}) recalage".format(x, y, z))
                pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["recalage"]))

            else:
                if self.forward:
                    if self.accurateMode:
                        LOG_INFO("\nDisplacement request ({}, {}, {}) accurateAv".format(x, y, z))
                        pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["accurateAv"]))
                    else:
                        LOG_INFO("\nDisplacement request ({}, {}, {}) dispAv".format(x, y, z))
                        pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["disp"]))
                
                else:
                    if self.accurateMode:
                        LOG_INFO("\nDisplacement request ({}, {}, {}) accurateAr".format(x, y, z))
                        pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["accurateAr"]))
                    else:
                        LOG_INFO("\nDisplacement request ({}, {}, {}) dispAr".format(x, y, z))
                        pub_teensy.publish(Quaternion(x, y, z, CMD_TEENSY["dispAr"]))

                pub_speed.publish(Float32MultiArray(self.maxSpeedLin, self.maxSpeedRot))
                self.move = True
            
        # Sinon, on on a fini (ou bien a nulle part ou aller, pcq obstacle
        # detecte sans qu'on bouge...)
        else:
            LOG_INFO("Arrived at destination!")
            # Reset des params
            self.move = False
            self.avoidMode = False
            self.recalageMode = False
            self.accurateMode = False

            # Publication a la strat
            pub_strat.publish(Int16(COM_STRAT["ok pos"]))


    # def setAvoidResetPoint(self):
    #     """Calcul du point de reset des marges d'evitement.
        
    #     Il s'agit du point à partir duquel on sera suffissament loin de
    #     l'obstacle jusqu'a la fin du trajet."""

    #     avoidRobotPos = self.pathfinder.getRobotToAvoidPos()[0]
    #     posList = [self.current_pos] + self.path
    #     i = len(posList) -1
    #     work = True
    #     while i > 0 and work: 
    #         #Parcours des droites de trajectoire de la fin vers le début
    #         if min(posList[i-1][0], posList[i][0])<avoidRobotPos[0]<max(posList[i-1][0], posList[i][0]) or min(posList[i-1][1], posList[i][1])<avoidRobotPos[1]<max(posList[i-1][1], posList[i][1]):
    #             if posList[i][0]-posList[i-1][0] !=0:
    #                 a = float(posList[i][1]-posList[i-1][1])/float(posList[i][0]-posList[i-1][0])
    #                 b=-1
    #                 c = posList[i-1][1]-a*posList[i-1][0]
    #             else:
    #                 a=1
    #                 b=0
    #                 c=-posList[i-1][0]
    #             #Calcul de la distance de la droite au centre de l'obstacle
    #             d = (a*avoidRobotPos[0]+b*avoidRobotPos[1]+c)/sqrt(a**2+b**2)
    #             if abs(d)<600:
    #                 work = False
    #         i-=1

    #     if work:    # On est toujours trop pres --> reset sur le point final
    #         self.resetPoint = self.path[-1][:2]
    #     else:       # On calcul et setup le point de reset
    #         vectN = np.array([a,b])/np.linalg.norm([a,b])
    #         self.resetPoint = np.array(avoidRobotPos) - d*vectN
            
    # def handle_obstacle(self, obstacle_seen, obstacle_stop, isDestBlocked):
    #     """Gestion d'arret du robot."""
    #     # dprint("Enter handle_obstacle()")
    #     if obstacle_stop:
    #         self.time_last_seen_obstacle = time()
    #         if not self.paused:
    #             LOG_INFO("New obstacle detected! - STOP")
    #             self.paused = True
    #             pub_teensy.publish(Quaternion(0,0,0,CMD_TEENSY["stop"]))
    #             if isDestBlocked:
    #                 LOG_ERRS("Destination is being blocked.")
    #                 pub_strat.publish(Int16(COM_STRAT["stop blocked"]))
    #             else:
    #                 pub_strat.publish(Int16(COM_STRAT["stop"]))
    #         return

    #     if obstacle_seen:   # Il faut ralentir
    #         # dprint("Obstacle is seen")
    #         # self.time_last_seen_obstacle = time()

    #         LOG_INFO("New obstacle detected! - SPEED DOWN")
    #         self.paused = False
    #         pub_teensy.publish(Quaternion(0,0,0,CMD_TEENSY["stop"]))
    #         pub_strat.publish(Int16(COM_STRAT["go"]))
    #         return

    #     if self.paused and (time()-self.time_last_seen_obstacle > self.refresh_update_obstacle):
    #         self.paused = False
    #         self.resume = True
    #         LOG_INFO("Resume displacement.")
    #         pub_strat.publish(Int16(COM_STRAT["go"]))
    #         self.next_point(False)



#################################################################
#																#
# 							MAIN PROG 							#
#																#
#################################################################

def main():
    # Init and create DecisionNode
    rospy.init_node("DisplacementNode")
    node = DisplacementNode()

    init_comm(node)
    init_gain(node)
    
    # Wait for close key to quit
    rospy.spin()


#################################################################
if __name__ == '__main__':
    main()