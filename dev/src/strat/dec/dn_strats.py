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
from dn_comm  import next_action_pub, stop_IT, score_pub, end_pub
from dn_utils import log_info, log_warn, log_errs, log_fatal
import numpy as np

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import Action, ActionScore, PLANTS_POS as PLANTS_POS_RAW, POTS_POS as POTS_POS_RAW

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

p_dn = None
PLANT_CAPACITY = 6
PLANT_THRESHOLD = 0
remaining_plants = [6 for _ in range(6)]
PLANTS_POS = np.array(PLANTS_POS_RAW)
POTS_POS = np.array(POTS_POS_RAW)[:, :2]

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
    time.sleep(0.01)

    if p_dn is None: # safety if the function is called before DEC node init (not supposed to happen)
        log_fatal("p_dn None in dn_strats, not supposed to happen")
        return


    if p_dn.go_park :
        p_dn.nb_actions_done[0] = 1  # jump to park action


    if p_dn.nb_actions_done[0] == 0:
        
        plant_id = np.argmin(np.linalg.norm(np.array(p_dn.position)[:2] - PLANTS_POS, axis=1))

        p_dn.curr_action = [Action.PICKUP_PLANT, plant_id]
        log_info("Next action order : Pickup Plants")
        publishAction()
        return


    if p_dn.nb_actions_done[0] == 1:

        pot_id = np.argmin(np.linalg.norm(np.array(p_dn.position)[:2] - POTS_POS, axis=1))

        p_dn.curr_action = [Action.PICKUP_POT, pot_id]
        log_info("Next action order : Pickup Pots")
        publishAction()        
        return
    
    
    if p_dn.nb_actions_done[0] == 2:
        p_dn.curr_action = [Action.PARK]
        log_info("Next action order : Park")
        publishAction()
        
        # Add to score because we earned points
        p_dn.score += ActionScore.SCORE_PARK.value
        publishScore()
        return
    
    
    if p_dn.nb_actions_done[0] == 3:
        p_dn.nb_actions_done[0] = -1  # to prevent repeated end action #BUG bof
        log_info("End of strategy : TEST")
        stop_IT()
        return



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
