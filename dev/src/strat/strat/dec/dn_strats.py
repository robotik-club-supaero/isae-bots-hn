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

from ..strat_const import Action, ActionScore, ActionResult

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################


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
    node.curr_action = [Action.PICKUP, 2]
    node.publishAction()
    node.get_logger().info(f"Next action order : Pick Up -> Box n°{2}")
    return

def homologation(node):
    """
    DN Strat: homologation
    
    Actions of this strategy :
        - 
        -
        -
    """
    last_action = node.curr_action[0]
    outcome = node.action_successful

    if last_action == Action.PICKUP and outcome == ActionResult.SUCCESS :
        node.curr_action = [Action.PARK] # just go park 
    else:
        node.curr_action = [Action.PICKUP, 1] # just go pickup box id n°1

    node.publishAction()
    return


def match_strat(node):
    """
    DN Strat: match (used for reach matches)
    
    Modify : node.curr_action -> ex: [Action.PICKUP, 3] -> Pickup element at index n°3

    """
    
    action_order = [Action.PICKUP, Action.DEPOSIT, Action.PICKUP, Action.CURSOR, Action.DEPOSIT, Action.PICKUP, Action.DEPOSIT, Action.PARKSTANDBY]
    
    def find_closest(node, positions, remaining, cond=None, coeffs=None, pos_type='boxes'):

        if coeffs is None:
            coeffs = [1 for _ in range(len(positions))]
        
        if cond is None: 
            cond = lambda remaining_index: remaining[remaining_index] > 0

        x, y, _ = node.position

        if pos_type == 'boxes':
            dists = [ coeffs[i] * ((x_p - x)**2 + (y_p - y)**2) for i, ((x_p, y_p, t_p), box_id) in enumerate(positions)]
        else:
            dists = [ coeffs[i] * ((x_p - x)**2 + (y_p - y)**2) for i, (x_p, y_p, t_p) in enumerate(positions)]

        dist_sorted_index = list(np.argsort(dists)) # Tri les distance du plus petit au plus grand et renvoi la liste trié des indexes de dists
        #print("\n\n" + str(positions) + "\n" + str(remaining) + "\n\n")
        for index in dist_sorted_index:
            element_id = positions[index][1] if pos_type == 'boxes' else index # If pos_type=box -> differentes positions pour une même box
            if cond(element_id):
                return element_id
        
        return None
    
    last_action = node.curr_action[0]
    time.sleep(0.01) # is it really necessary ?

    def deposit_closest():
        DEPOSIT_POS = node.config.deposit_zones_pos
        malus_deposit = np.ones(len(DEPOSIT_POS))   
        deposit_id = find_closest(node, DEPOSIT_POS, node.remaining_deposits_slots, coeffs=malus_deposit, pos_type='deposit')
        if deposit_id is not None:
            return True, deposit_id
        return False, None
    
    def pickup_closest():
        BOXES_POS = node.config.pickup_boxes_pos
        malus_pickup = np.ones(len(BOXES_POS))
        box_id = find_closest(node, BOXES_POS, node.remaining_boxes_areas, coeffs=malus_pickup, pos_type='boxes')
        if box_id is not None:
            return True, box_id
        return False, None

    def set_next_action():
        next_action = action_order[node.action_step_index]

        if next_action == Action.PARK:
            node.curr_action = [Action.PARK]
            node.get_logger().info("Next action order : Park")
            return True
        
        if next_action == Action.WAIT:
            node.curr_action = [Action.WAIT]
            node.get_logger().info(f"Next action order : Wait")        
            return True

        if next_action == Action.CURSOR:
            node.curr_action = [Action.CURSOR]
            node.get_logger().info(f"Next action order : Push Cursor")        
            return True
        
        if next_action == Action.PICKUP:
            can_pickup, box_id = pickup_closest()
            if can_pickup:
                node.curr_action = [Action.PICKUP, box_id]
                node.get_logger().info(f"Next action order : Pick Up -> Box n°{box_id}")
                return True
            else:
                node.get_logger().info("No more box to pick up")      
        
        if next_action == Action.DEPOSIT:
            can_deposit, deposit_id = deposit_closest()
            if can_deposit:
                node.curr_action = [Action.DEPOSIT, deposit_id]
                node.get_logger().info(f"Next action order : Deposit -> Area n°{deposit_id}")
                return True
            else:
                node.get_logger().info("No more slot to deposit.")
        
        if next_action == Action.PARKSTANDBY:
            node.curr_action = [Action.PARKSTANDBY]
            node.get_logger().info(f"Next action order : Park Standby")        
            return True

        return False

    # Init
    if last_action == Action.INIT:
        node.action_step_index = 0 # Initialise to first action
        success = set_next_action()
        if success:
            node.get_logger().info(f"Next action order : Beginning of the startegy -> {node.curr_action}")
            node.publishAction()
            return
        else:
            node.get_logger().info(f"Next Action : First Action not recognised. -> {action_order[node.action_step_index]}")

    # Pending
    if last_action == Action.PENDING:
        node.get_logger().info(f"Action PENDING.. waiting for trigger.")
        node.publishAction()
        return

    # Retry
    if not node.action_successful and not node.go_park:
        if node.retry_count < 2: # retry une fois
            node.get_logger().info(f"DN asked Strategy for next action while last action not succeed : {node.curr_action[0]} -> RETRY.")
            node.retry_count += 1 # reset in dec_node when action success
            success = set_next_action()
            if success:
                node.publishAction()
                return
            else:
                node.get_logger().info(f"Retry Failed : Action not recognised. ({action_order[node.action_step_index]})")

    
    if not node.go_park:

        # If failed + retry failed too -> Debug Print
        if not node.action_successful:
            node.get_logger().info(f"ACTION '{last_action}' FAILED and RETRY FAILED. Skipping to next action.")

        node.action_step_index += 1 # Go to next action
        if (node.action_step_index) >= len(action_order):
            node.get_logger().info(f"End of Action Order defined by Strategy. -> STOP")   
            node.get_logger().info("End of strategy : MATCH")
            node.stop_IT() 
            return

        success = set_next_action()
        if success:
            node.publishAction()
            return
        else:
            node.get_logger().info(f"Next Action : Action not recognised. ({action_order[node.action_step_index]})")
            node.get_logger().info("Out of Normal Loop : switching to Pickup")
            can_pickup, pickup_id = pickup_closest()
            if can_pickup:
                node.get_logger().info(f"Next action order : Picking Up -> Box n°{pickup_id}")
                node.publishAction()
                return
            else:
                node.get_logger().info("No more box to pick up")
        
        # Go to park
        node.get_logger().info(f"Strategy could not find any solution after : action={str(node.curr_action)}, success={node.action_successful}")
        node.curr_action = [Action.PARK]
        node.get_logger().info("Next action order : Park")
        node.publishAction()
        return

    # End of Match
    if node.parked:
        node.get_logger().info("End of strategy : MATCH")
        node.stop_IT() # Stop robot
        return
    
    # Go to park
    node.curr_action = [Action.PARK]
    node.get_logger().info("Next action order : Park")
    node.publishAction()
    return
