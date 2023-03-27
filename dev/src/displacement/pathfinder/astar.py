#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: astar.py
@status: in progress

Librairie implementant un algorithme de A* sur un ensemble de noeud et 
d'obstacle définie dans la classe Map.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import time

from node import Node
from Queue import PriorityQueue
from exceptions import PathNotFoundError, TimeOutError

### CONSTANTES ########################################################
X_THRESHOLD = 5     # 5mm
Y_THRESHOLD = 5     # 5mm
#######################################################################

def isNodeInList(list, node):
    #Renvoie True si le noeud demandé est dans la liste et False sinon
    for testNode in list:
        if testNode.equals(node):
            return True
    return False

def isNodeOutObstacles(tableMap, node, isFirst):
    #Teste si le noeud est en dehors des obstacles
    if not tableMap.getAvoid():
        return True
    if not isFirst:
        return True
    for obstacle in tableMap.getObstacleList():
        if obstacle.isNodeIn(node):
            return False
    return True


#######################################################################
#
#                           Algo A star
#
#######################################################################
    
def a_star(init, goal, tableMap, isFirstAccurate, maxAstarTime):
    """Algorithme du A* renvoyant un chemin entre start et goal"""

#######################################################################
# PHASE D'INITIALISATION
#######################################################################
    
    # Le noeud de test courant est initialise au noeud de depart
    goalNode = Node(goal)
    currNode = Node(init)    

    # Si l'arrivee est visible depuis le départ, on renvoie directement un chemin vers l'arrivée
    if goalNode.isVisible(currNode, tableMap):
        return [goalNode.getPosition()]
    
    currNode.setInitDist(0)
    isFirstAccurate = not isNodeOutObstacles(tableMap, currNode, True)
    isInAvoidMode = tableMap.getAvoid()
    
    # Init liste du A*
    openedList = PriorityQueue() ## La queue des noeuds à regarder
    openedList.put((currNode.weight(), currNode))
    closedList = [] ## La liste des noeuds déjà visités
    
    # Liaison des noeuds de depart et d'arrivee avec les noeuds de la carte
    bestNode = None
    bestDist = 0
    for node in tableMap.getNodeList():
        goalDist = node.distFromNode(goalNode)

        # Lors d'un evitement on ne connecte le noeud d'arrive qu'aux noeud adjacent
        if isInAvoidMode and goalDist < 200:  
            node.addLinkNodeList(goalNode)
        else:
            node.addLinkNodeList(goalNode)

        # On connecte le noeud de départ
        initDist = node.distFromNode(currNode)

        if bestNode == None:
            bestNode = node
            bestDist = initDist

        if initDist < 1500:
            # On rajoute le noeud s'il est visible et hors des obstacles
            if currNode.isVisible(node, tableMap) and isNodeOutObstacles(tableMap, node, False):
                currNode.addLinkNodeList(node)

            # On garde le noeud même si la liaison traverse un obstacle si jamais on avait pas de meilleur noeud
            if isNodeOutObstacles(tableMap, node, False):
                if initDist < bestDist:
                    bestNode = node
                    bestDist = initDist

    # Si la liste de connection du noeud de depart est vide on force la premiere connection avec le noeud le plus proche          
    if currNode.getLinkNodeList() == [] and bestNode != None:
        bestNode.setGoalDist(bestNode.distFromNode(goalNode))
        bestNode.updateNode(currNode)

#######################################################################
# PHASE DE RECHERCHE
#######################################################################

    begin = time.time()

    # Boucle de parcours du graphe selon l'algo de A*
    while not openedList.empty():
        # NB: A peu pres 1000 passages ici quand on ne trouve pas de path (depend de la map et des positions)
        if time.time() - begin > maxAstarTime:  # interruption de l'Astar
            raise TimeOutError
        
        currNode = openedList.get()
        closedList.append(currNode)

        if currNode == goalNode :
            break

        for neighbor in currNode.getLinkNodeList() :

            if not neighbor.isVisible(currNode, tableMap):  # on garanti que le noeud est visible depuis currNode
                continue
            if isNodeInList(closedList, neighbor):          # on garanti qu'il n'est pas dans la liste fermé
                continue

            neighbor.setGoalDist(neighbor.distFromNode(goalNode))

            new_cost = currNode.getWeight() + neighbor.getGoalDist() + neighbor.distFromNode(currNode)

            if neighbor.getWeight() == None or neighbor.getWeight() > new_cost:
                neighbor.updateNode(currNode)
                openedList.put((neighbor.getWeight(), neighbor))

#######################################################################
# POST TRAITEMENT DU PATH
#######################################################################

    # On récupère le chemin (à l'envers) (foundPath = [goal, ..., init])
    foundPath = []
    while currNode.parent != None:
        foundPath.append(currNode)
        currNode = currNode.getParent()
    foundPath.append(currNode)
    
    finalPath = []
    nbPts = len(foundPath)

    # Sinon on parcours la liste à l'envers et supprime les points intermediaire si inutiles (autres points visibles)
    for i in range(nbPts-1, 1, -1):
        if foundPath[i].isVisible(foundPath[i-2], tableMap):
            del foundPath[i-1]
            continue
        if (i < nbPts-1):
            finalPath.append(foundPath[i].getPosition())

    finalPath.append(foundPath[1].getPosition())
    finalPath.append(foundPath[0].getPosition())

    return finalPath