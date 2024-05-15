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

import os, sys, inspect
import time
from dn_comm  import next_action_pub, stop_IT, score_pub, end_pub, PLANTS_POS, POTS_POS, DEPOSIT_POS, PARK_POS
from dn_utils import log_info, log_warn, log_errs, log_fatal
import numpy as np

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import Action, ActionScore
from strat_utils import adapt_pos_to_side

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

p_dn = None
PLANT_THRESHOLD = 0

def init_strats(dn):
    global p_dn
    p_dn = dn

#################################################################
#                                                               #
#                           STRATS                              #
#                                                               #
#################################################################

def publishAction():
    log_info(p_dn.curr_action)
    p_dn.action_successful = False
    p_dn.retry_count += 1
    next_action_pub.publish(data = [p_dn.curr_action[0].value] + p_dn.curr_action[1:])
    

def test_strat():
    """
    DN Strat: test
    
    Actions of this strategy :
        -
        -
        -
    """
    pass

def homologation():
    """
    DN Strat: homologation
    
    Actions of this strategy :
        - 
        -
        -
    """
    
   # if p_dn.go_park or p_dn.solar_panels[2]:
   #     p_dn.curr_action = [Action.PARK, 0]
  #  if not p_dn.solar_panels[0]:
   #     p_dn.curr_action = [Action.TURN_SOLAR_PANEL, 0]
  #  elif not p_dn.solar_panels[1]:
  #      p_dn.curr_action = [Action.TURN_SOLAR_PANEL, 1]
   # elif not p_dn.solar_panels[2]:
  #      p_dn.curr_action = [Action.TURN_SOLAR_PANEL, 2]

 #   publishAction()
 #   return

   # p_dn.nb_actions += 1
   # if p_dn.nb_actions == 1:
   #     p_dn.curr_action = [Action.PICKUP_PLANT, 4]

   # if p_dn.nb_actions == 2:
    p_dn.curr_action = [Action.PARK, 2]

    publishAction()


def match_strat():
    """
    DN Strat: match (used for reach matches)
    
    Actions of this strategy :
        - 
        -
        -
    """
    
    def find_closest(p_dn, positions, remaining, cond=None, relative=True, coeffs=1):
        if cond is None: cond = lambda cluster: remaining[cluster] > PLANT_THRESHOLD
        x, y, _ = adapt_pos_to_side(*p_dn.position, p_dn.color) if relative else p_dn.position
        dists = coeffs * np.linalg.norm(np.array([x,y]) - positions, axis=1)
        clusters = np.argsort(dists)
        for cluster in clusters:
            if cond(cluster.item()):
                return cluster
        return None

    time.sleep(0.01)

    if p_dn is None: # safety if the function is called before DEC node init (not supposed to happen)
        log_fatal("p_dn None in dn_strats, not supposed to happen")
        return

    if not p_dn.go_park:

        if p_dn.curr_action[0] == Action.TURN_SOLAR_PANEL:
            if p_dn.curr_action[1] < 5 and p_dn.action_successful:
                p_dn.curr_action[1] += 1
                publishAction()
                return
            # else change action, retry later

        coeffs = np.ones(6)
        if p_dn.curr_action[0] != Action.PENDING and not p_dn.action_successful:
            if p_dn.retry_count < 3:
                publishAction()
                return
            elif len(p_dn.curr_action) >= 2:
                coeffs[p_dn.curr_action[1]] = 999

        if (p_dn.curr_action[0] == Action.PICKUP_PLANT and p_dn.action_successful) or (p_dn.curr_action[0] == Action.PICKUP_POT and not p_dn.action_successful):
            pot_id = find_closest(p_dn, POTS_POS, p_dn.remaining_pots, coeffs=coeffs)
            if pot_id is not None:
                p_dn.curr_action = [Action.PICKUP_POT, pot_id]
                log_info("Next action order : Pickup Pots")
                publishAction()        
                return
            else:
                log_info("No more pot to pick up")
            return

        if (p_dn.curr_action[0] == Action.PICKUP_POT and p_dn.action_successful) or (p_dn.curr_action[0] == Action.DEPOSIT_POT and not p_dn.action_successful):
            pot_id = find_closest(p_dn, DEPOSIT_POS, p_dn.deposit_slots, coeffs=[1,1,0.3]*coeffs[:3]) # deposit in secure area first
            if pot_id is not None:
                p_dn.curr_action = [Action.DEPOSIT_POT, pot_id]
                log_info("Next action order : Deposit Pots")
                publishAction()        
                return
            else:
                log_info("No more free slot to deposit")

        if p_dn.curr_action[0] == Action.DEPOSIT_POT:
            for i in range(6):
                if not p_dn.solar_panels[i]:
                    p_dn.curr_action = [Action.TURN_SOLAR_PANEL, i]
                    publishAction()
                    return

        # If no other action is applicable, defaulting to picking up plant

        plant_id = find_closest(p_dn, PLANTS_POS, p_dn.remaining_plants, coeffs=coeffs)
        if plant_id is not None:
            p_dn.curr_action = [Action.PICKUP_PLANT, plant_id]
            log_info("Next action order : Pickup Plants")
            publishAction()
            return
        else:
            log_info("No more plant to pick up")

    if p_dn.parked:
        log_info("End of strategy : MATCH")
        stop_IT()
        return
    
    # If no other action is applicable, go to park

    zone = find_closest(p_dn, PARK_POS, None, cond=lambda index: index != p_dn.init_zone)
    p_dn.curr_action = [Action.PARK, zone]
    log_info("Next action order : Park")
    publishAction()

    pass
