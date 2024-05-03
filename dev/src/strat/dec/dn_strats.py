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
    next_action_pub.publish(data = [p_dn.curr_action[0].value] + p_dn.curr_action[1:])
    
    
def publishScore():
    score_pub.publish(data=p_dn.score)
    

def test_strat():
    """
    DN Strat: test
    
    Actions of this strategy :
        -
        -
        -
    """

    def find_closest(p_dn, positions, remaining, cond=None, relative=True):
        if cond is None: cond = lambda cluster: remaining[cluster] > PLANT_THRESHOLD
        x, y, _ = adapt_pos_to_side(*p_dn.position, p_dn.color) if relative else p_dn.position
        dists = np.linalg.norm(np.array([x,y]) - positions, axis=1)
        clusters = np.argsort(dists)
        for cluster in clusters:
            if cond(cluster.item()):
                return cluster
        return None
    
    time.sleep(0.01)

    if p_dn is None: # safety if the function is called before DEC node init (not supposed to happen)
        log_fatal("p_dn None in dn_strats, not supposed to happen")
        return

    publishScore()
    
    if not p_dn.go_park:
        if p_dn.nb_actions_done[0] == 0:
            p_dn.curr_action = [Action.TURN_SOLAR_PANELS]
            publishAction()  
            return

        if p_dn.nb_actions_done[0] == 1 or (p_dn.nb_actions_done[0] == 4 and not p_dn.go_park):

            plant_id = find_closest(p_dn, PLANTS_POS, p_dn.remaining_plants)
            if plant_id is not None:
                p_dn.nb_actions_done[0] = 1
                p_dn.curr_action = [Action.PICKUP_PLANT, plant_id]
                log_info("Next action order : Pickup Plants")
                publishAction()
                return
            else:
                log_info("No more plant to pick up")


        if p_dn.nb_actions_done[0] == 2:

            pot_id = find_closest(p_dn, POTS_POS, p_dn.remaining_pots)
            if pot_id is not None:
                p_dn.curr_action = [Action.PICKUP_POT, pot_id]
                log_info("Next action order : Pickup Pots")
                publishAction()        
                return
            else:
                log_info("No more pot to pick up")

        
        if p_dn.nb_actions_done[0] == 3:

            pot_id = find_closest(p_dn, DEPOSIT_POS, p_dn.deposit_slots)
            if pot_id is not None:
                p_dn.curr_action = [Action.DEPOSIT_POT, pot_id]
                log_info("Next action order : Deposit Pots")
                publishAction()        
                return
            else:
                log_info("No more free slot to deposit")

    if p_dn.parked:
        log_info("End of strategy : TEST")
        stop_IT()
        return

    zone = find_closest(p_dn, PARK_POS, None, cond=lambda index: index != p_dn.init_zone)
    p_dn.curr_action = [Action.PARK, zone]
    log_info("Next action order : Park")
    publishAction()

def homologation():
    """
    DN Strat: homologation
    
    Actions of this strategy :
        - 
        -
        -
    """
    
    time.sleep(0.01)

    pass


def match_strat():
    """
    DN Strat: match (used for reach matches)
    
    Actions of this strategy :
        - 
        -
        -
    """
    time.sleep(0.01)

    pass
