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
from dn_comm  import next_action_pub, stop_IT, take_cakes_pub, deposit_cakes_pub
from dn_utils import log_info, LIST_OF_ACTIONS

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

def test_strat():
    """
    DN Strat: homologation
    
    Note: Define below the actions of this strategy 
        -
        -
        -
    """
    time.sleep(0.01)

    #if p_dn is None: return  # safety if the function is called before DEC node init TODO : not supposed to happen ?

    if p_dn.nb_actions_done[0] == 0:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCherriesPerpendicular']
        log_info("Next action : Take Cherries Perpendicular")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 1:
        p_dn.curr_action = LIST_OF_ACTIONS['depositCherries']
        log_info("Next action : Deposit Cherries")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 2:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(0)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 3:
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(0)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 4:
        p_dn.curr_action = LIST_OF_ACTIONS['park']
        log_info("Next action : Park")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 5:
        p_dn.nb_actions_done[0] =-1  # to prevent repeated end action
        log_info("End of strategy : HOMOLOGATION")
        stop_IT()
        return


def tests_strat():
    """
    DN Strat: tests (for all tests before the cup)
    
    Note: Define below the actions of this strategy
        - 
        -
        -
    """
    time.sleep(0.01)

    log_info("Entered tests_strat section")


    return


def match_strat():
    """
    DN Strat: match (only for the cup)
    
    Note: Define below the actions of this strategy
        - 
        -
        -
    """
    time.sleep(0.01)

    pass
