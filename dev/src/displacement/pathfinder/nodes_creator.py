#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: nodes_creator.py
@status: OK
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import os

import xml.etree.ElementTree as ET
from node import Node

from disp_utils import ROBOT_NAME, READER

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def makeNodeList(option):
    """
    Function to create a list of nodes for the pathfinder grid of nodes.
    
    Parameters 
    ----------
    option: [int] | -1, 0, ..., n 
        Specifies the type of map that must be used (avoiding, config1, config2, ...).
        In the case of the years before 2023, there were 2 config : HOME and AWAY.
        In the case of the year 2023, there were multiple config depending of the start position (but always two colors).
    
    Return
    ------
    nodelist: [list(Node)]
        A list of Node objects - the node list for the map.
    """
  
    ## CONFIG DATA ####################################################

    number_config = int(READER.get("Robot", "number_config"))

    if option == -1:
        data = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_avoiding.xml"))

    elif option < number_config:
        data = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_config"+str(option)+".xml"))

    else:
        raise RuntimeError("Specified option is invalid")

    ## BUILDING NODES #################################################
    nodeList = []
    for nodeParam in data.xpath("/map/node"):
        nodeList.append(Node([int(nodeParam.get("x")),int(nodeParam.get("y"))])) 
    for linkParam in data.xpath("/map/connection"):
        node1Index = int(linkParam.get("p1")[1:])
        node2Index = int(linkParam.get("p2")[1:])
        nodeList[node1Index].addLinkNodeList(nodeList[node2Index])
        nodeList[node2Index].addLinkNodeList(nodeList[node1Index])
    return nodeList
