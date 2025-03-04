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
from .dn_const import PLANTS_POS, POTS_POS, DEPOSIT_POS, PARK_POS
import numpy as np

from ..strat_const import Action, ActionScore
from ..strat_utils import adapt_pos_to_side

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

PLANT_THRESHOLD = 0

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
    
   # if node.go_park or node.solar_panels[2]:
   #     node.curr_action = [Action.PARK, 0]
  #  if not node.solar_panels[0]:
   #     node.curr_action = [Action.TURN_SOLAR_PANEL, 0]
  #  elif not node.solar_panels[1]:
  #      node.curr_action = [Action.TURN_SOLAR_PANEL, 1]
   # elif not node.solar_panels[2]:
  #      node.curr_action = [Action.TURN_SOLAR_PANEL, 2]

 #   node.publishAction()
 #   return

   # node.nb_actions += 1
   # if node.nb_actions == 1:
   #     node.curr_action = [Action.PICKUP_PLANT, 4]

   # if node.nb_actions == 2:
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
        if cond is None: cond = lambda cluster: remaining[cluster] > PLANT_THRESHOLD
        x, y, _ = adapt_pos_to_side(*node.position, node.color) if relative else node.position
        dists = coeffs * np.linalg.norm(np.array([x,y]) - positions, axis=1)
        clusters = np.argsort(dists)
        for cluster in clusters:
            if cond(cluster.item()):
                return cluster
        return None

    time.sleep(0.01)

    if not node.go_park:

        if node.curr_action[0] == Action.TURN_SOLAR_PANEL:
            if node.curr_action[1] < 5 and node.action_successful:
                node.curr_action[1] += 1
                node.publishAction()
                return
            # else change action, retry later

        coeffs = np.ones(6)
        if node.curr_action[0] != Action.PENDING and not node.action_successful:
            if node.retry_count < 3:
                node.publishAction()
                return
            elif len(node.curr_action) >= 2:
                coeffs[node.curr_action[1]] = 999 # penalise

        if (node.curr_action[0] == Action.PICKUP_PLANT and node.action_successful) or (node.curr_action[0] == Action.PICKUP_POT and not node.action_successful):
            pot_id = find_closest(node, POTS_POS, node.remaining_pots, coeffs=coeffs)
            if pot_id is not None:
                node.curr_action = [Action.PICKUP_POT, pot_id]
                node.get_logger().info("Next action order : Pickup Pots")
                node.publishAction()        
                return
            else:
                node.get_logger().info("No more pot to pick up")
            return

        if (node.curr_action[0] == Action.PICKUP_POT and node.action_successful) or (node.curr_action[0] == Action.DEPOSIT_POT and not node.action_successful):
            pot_id = find_closest(node, DEPOSIT_POS, node.deposit_slots, coeffs=[1,1,0.3]*coeffs[:3]) # deposit in secure area first
            if pot_id is not None:
                node.curr_action = [Action.DEPOSIT_POT, pot_id]
                node.get_logger().info("Next action order : Deposit Pots")
                node.publishAction()        
                return
            else:
                node.get_logger().info("No more free slot to deposit")

        if node.curr_action[0] == Action.DEPOSIT_POT:
            for i in range(6):
                if not node.solar_panels[i]:
                    node.curr_action = [Action.TURN_SOLAR_PANEL, i]
                    node.publishAction()
                    return

        # If no other action is applicable, defaulting to picking up plant

        plant_id = find_closest(node, PLANTS_POS, node.remaining_plants, coeffs=coeffs)
        if plant_id is not None:
            node.curr_action = [Action.PICKUP_PLANT, plant_id]
            node.get_logger().info("Next action order : Pickup Plants")
            node.publishAction()
            return
        else:
            node.get_logger().info("No more plant to pick up")

    if node.parked:
        node.get_logger().info("End of strategy : MATCH")
        node.stop_IT()
        return
    
    # If no other action is applicable, go to park

    zone = find_closest(node, PARK_POS, None, cond=lambda index: index != node.init_zone)
    node.curr_action = [Action.PARK, zone]
    node.get_logger().info("Next action order : Park")
    node.publishAction()

    pass
