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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import time
from .dn_const import STAND_POS, DEPOSIT_POS, PARK_POS
import numpy as np

from ..strat_const import Action, ActionScore
from ..strat_utils import adapt_pos_to_color

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

STAND_THRESHOLD = 0

#################################################################
#                                                               #
#                           STRATS                              #
#                                                               #
#################################################################


def test_strat(node):
    """
    DN Strat: test
    
    Actions of this strategy :
        -
        -
        -
    """
    pass

def homologation(node):
    """
    DN Strat: homologation
    
    Actions of this strategy :
        - 
        -
        -
    """
    node.curr_action = [Action.PARK, 2]

    node.publishAction()


def match_strat(node):
    """
    DN Strat: match (used for reach matches)
    
    Actions of this strategy :
        - 
        -
        -
    """
    
    def find_closest(node, positions, remaining, cond=None, relative=True, coeffs=1):

        if cond is None: cond = lambda cluster: remaining[cluster] > STAND_THRESHOLD

        x, y, _ = adapt_pos_to_color(*node.position, node.color) if relative else node.position
        print("\n#### Position : ", positions, "##### \n")
        dists = [ coeffs * ((x_p * x + y_p * y)**0.5) for x_p, y_p in positions]

        clusters = np.argsort(dists)
        for cluster in clusters:
            if cond(cluster.item()):
                return cluster
            
        return None

    time.sleep(0.01)

    if not node.go_park:
        
        # Retry
        if node.curr_action[0] != Action.PENDING and not node.action_successful:
            if node.retry_count < 3:
                node.publishAction()
                return

        # Pickup Up Stand
        malus_pickup = np.ones(len(STAND_POS))
        if (node.curr_action[0] == Action.PICKUP_STAND_1 or node.curr_action[0] == Action.PICKUP_STAND_2):

            if not node.action_successful:
                malus_pickup[node.curr_action[1]] = 999 # penalise
            
            stand_id = find_closest(node, STAND_POS, node.remaining_stands, coeffs=malus_pickup)
            
            if stand_id is not None:
                if not node.action_successful: # Continue to search for stand                
                    node.curr_action[1] = stand_id        
                elif node.curr_action[0] == Action.PICKUP_STAND_2: # top stand picked
                    node.remaining_stands[node.curr_action[1]] = 0
                    node.curr_action = [Action.PICKUP_STAND_2, stand_id]
                elif node.curr_action[0] == Action.PICKUP_STAND_1: # bottom stand picked
                    node.remaining_stands[node.curr_action[1]] = 0
                    node.curr_action = [Action.PICKUP_STAND_1, stand_id]
                
                node.get_logger().info(f"Next action order : Pickup Stand n°{stand_id}")
                node.publishAction()
                return
            else:
                node.get_logger().info("No stand found to pick up !")
        
        # Deposit Stand
        malus_deposit = np.ones(len(DEPOSIT_POS))
        if (not node.action_successful and node.curr_action[0] == Action.DEPOSIT_STAND):
            deposit_id = find_closest(node, DEPOSIT_POS, node.deposit_slots, coeffs=malus_deposit)
            if deposit_id is not None:
                node.curr_action = [Action.DEPOSIT_STAND, deposit_id]
                node.get_logger().info(f"Next action order : Deposit Stand at n°{deposit_id}")
                node.publishAction()        
                return
            else:
                node.get_logger().info("No more free slot to deposit")

        # If no other action is applicable, defaulting to picking up stand
        stand_id = find_closest(node, STAND_POS, node.remaining_stands, coeffs=malus_pickup)
        if stand_id is not None:
            node.curr_action = [Action.PICKUP_STAND_2, stand_id]
            node.get_logger().info("Next action order : Pickup Stand")
            node.publishAction()
            return
        else:
            node.get_logger().info("No more stand to pick up")

    if node.parked:
        node.get_logger().info("End of strategy : MATCH")
        node.stop_IT()
        return
    
    # If no other action is applicable, go to park
    zone = adapt_pos_to_color(PARK_POS, node.color)
    node.curr_action = [Action.PARK, zone]
    node.get_logger().info("Next action order : Park")
    node.publishAction()
    return
