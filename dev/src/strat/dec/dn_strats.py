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
from dn_comm  import next_action_pub, stop_IT, score_pub, end_pub
from dn_utils import ACTIONS, ACTIONS_SCORE, log_info, log_warn, log_errs, log_fatal

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

p_dn = None

def init_strats(dn):
    global p_dn
    p_dn = dn

#################################################################
#                                                               #
#                           STRATS                              #
#                                                               #
#################################################################

def publishAction():
    next_action_pub.publish( [p_dn.curr_action.value] )
    
    
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
        p_dn.curr_action = ACTIONS.simpleAction
        log_info("Next action : Take Cherries Perpendicular")
        publishAction()
        return


    if p_dn.nb_actions_done[0] == 1:
        p_dn.curr_action = ACTIONS.park
        log_info("Next action : Park")
        publishAction()
        
        # Add to score because we earned points
        p_dn.score += ACTIONS_SCORE.parking.value
        publishScore()
        
        return
    
    
    
    if p_dn.nb_actions_done[0] == 2:
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
