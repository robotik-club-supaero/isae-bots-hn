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
import numpy as np

from ..strat_const import Action, ActionScore

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
    node.curr_action = [Action.PICKUP_STAND_1, 3]

    node.publishAction()


def match_strat(node):
    """
    DN Strat: match (used for reach matches)
    
    Actions of this strategy :
        - 
        -
        -
    """
    
    def find_closest(node, positions, remaining, cond=None, coeffs=1, pos_type='stand'):

        if cond is None: 
            cond = lambda remaining_index: remaining[remaining_index] > STAND_THRESHOLD

        x, y, _ = node.position

        if pos_type == 'stand':
            dists = [ coeffs[i] * ((x_p - x)**2 + (y_p - y)**2) for i, ((x_p, y_p, t_p), stand_id) in enumerate(positions)]
        else:
            dists = [ coeffs[i] * ((x_p - x)**2 + (y_p - y)**2) for i, (x_p, y_p, t_p) in enumerate(positions)]

        dist_sorted_index = list(np.argsort(dists))
        #print(coeffs, dists, dist_sorted_index)
        for index in dist_sorted_index:
            true_index = positions[index][1] if pos_type == 'stand' else index
            if cond(true_index):
                return index
            
        return None

    time.sleep(0.01)

    if not node.go_park:
        
        # Retry
        if node.curr_action[0] != Action.PENDING and not node.action_successful:
            if node.retry_count < 3:
                node.get_logger().info(f"Retry action order : {node.curr_action[0]}")        
                node.publishAction()
                return

        # Pickup Up Stand
        STAND_POS = node.config.pickup_stand_pos
        DEPOSIT_POS = node.config.deposit_pos

        malus_deposit = np.ones(len(DEPOSIT_POS))
        malus_pickup = np.ones(len(STAND_POS))

        # Pickup Up Stand
        if (node.curr_action[0] in (Action.PICKUP_STAND_1, Action.PICKUP_STAND_2)):

            if not node.action_successful:
                malus_pickup[node.curr_action[1]] = float('inf') # penalise
            
            stand_id = find_closest(node, STAND_POS, node.remaining_stands, coeffs=malus_pickup, pos_type='stand')
            
            if stand_id is not None:
                if not node.action_successful: # Continue to search for stand                
                    node.curr_action[1] = stand_id
                    node.get_logger().info(f"Continue action order : Picking up Stand n°{stand_id}...")        
                elif node.curr_action[0] == Action.PICKUP_STAND_2: # top stand picked -> Go bottom stand picking
                    node.remaining_stands[node.curr_action[1]] = 0
                    node.curr_action = [Action.PICKUP_STAND_1, stand_id]
                    node.get_logger().info(f"Next action order : STAND_1 -> Pickup Stand n°{stand_id}")
                elif node.curr_action[0] == Action.PICKUP_STAND_1: # bottom stand picked -> Go deposit
                    node.remaining_stands[node.curr_action[1]] = 0
                    deposit_id = find_closest(node, DEPOSIT_POS, node.deposit_slots, coeffs=malus_deposit, pos_type='deposit')
                    if deposit_id is not None:
                        node.curr_action = [Action.DEPOSIT_STAND, deposit_id]
                        node.get_logger().info(f"Next action order : Deposit Stand at n°{deposit_id}")      
                    else:
                        node.get_logger().info("No deposit pos found !")
                node.publishAction()        
                return
            else:
                node.get_logger().info("No stand found to pick up !")
        
        # Deposit Stand
        if node.curr_action[0] == Action.DEPOSIT_STAND:
            if node.action_successful:
                node.deposit_slots[node.curr_action[1]] = 0
            else:
                deposit_id = find_closest(node, DEPOSIT_POS, node.deposit_slots, coeffs=malus_deposit, pos_type='deposit')
                if deposit_id is not None:
                    node.curr_action = [Action.DEPOSIT_STAND, deposit_id]
                    node.get_logger().info(f"Next action order : Deposit Stand at n°{deposit_id}...")
                    node.publishAction()        
                    return
                else:
                    node.get_logger().info("No more free slot to deposit !")

        # If no other action is applicable, defaulting to picking up stand
        node.get_logger().info("Out of Normal Loop : switching to Pickup Stand 2")
        stand_id = find_closest(node, STAND_POS, node.remaining_stands, coeffs=malus_pickup, pos_type='stand')
        if stand_id is not None:
            node.curr_action = [Action.PICKUP_STAND_2, stand_id]
            node.get_logger().info(f"Next action order : STAND_2 -> Pickup Stand n°{stand_id}")
            node.publishAction()
            return
        else:
            node.get_logger().info("No more stand to pick up")

    if node.parked:
        node.get_logger().info("End of strategy : MATCH")
        node.stop_IT()
        return
    
    # If no other action is applicable, go to park
    node.curr_action = [Action.PARK]
    node.get_logger().info("Next action order : Park")
    node.publishAction()
    return
