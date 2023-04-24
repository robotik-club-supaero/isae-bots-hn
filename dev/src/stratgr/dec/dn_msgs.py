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

import os
import time
import rospy
import threading
from dn_utils import log_info, log_errs, log_warn, \
                    ROBOT_SIDES, STRAT_NAMES, STRAT_INDEX, CB_NEXT_ACTION, \
                    TERM_SIZE
from std_msgs.msg      import Empty, Int16, Int16MultiArray
from geometry_msgs.msg import Quaternion, Pose2D

from message.msg       import InfoMsg, ActionnersMsg, EndOfActionMsg

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

p_dn = None

def init_msgs(dn):
    """
    Init the decision node in this file (no double imports)
    """
    global p_dn
    p_dn = dn

    #init_pubs()
#################################################################
#                                                               #
#                           FEEDBACK                            #
#                                                               #
#################################################################

def start_match(msg):
    """
    Feedback on start signal /game/start
    """

    if p_dn is None: return  # safety if the function is called before DEC node init

    if p_dn.match_started: return

    if msg.data == 1:
        log_info('\033[1m\033[36m' + "#"*20 + " Start match " + "#"*20 + '\033[0m')

        p_dn.match_started = True
        p_dn.start_time = time.time()
        threading.Timer(p_dn.match_time - p_dn.delay_park, park_IT).start()
        threading.Timer(p_dn.match_time, stop_IT).start()


def setup_color(msg):
    """
    Feedback on color side /game/color.
    """
    if p_dn is None: return  # safety if the function is called before DEC node init

    p_dn.color = msg.data
    if p_dn.color == ROBOT_SIDES.HOME:
        log_info("Received color : HOME")
    else:
        log_info("Received color : AWAY")


def setup_strat(msg):
    """
    Feedback on strategy chosen /game/strat.*
    """
    if p_dn is None: return  # safety if the function is called before DEC node init

    if msg.data == STRAT_INDEX.HOMOLOGATION:
        p_dn.strat_index = STRAT_INDEX.HOMOLOGATION
        log_info(f"Received strat: {STRAT_NAMES.HOMOLOGATION}")
        return
    if msg.data == STRAT_INDEX.TESTS:
        p_dn.strat_index = STRAT_INDEX.TESTS
        log_info(f"Received strat: {STRAT_NAMES.TESTS}")
        return
    if msg.data == STRAT_INDEX.MATCH:
        p_dn.strat_index = STRAT_INDEX.MATCH
        log_info(f"Received strat: {STRAT_NAMES.MATCH}")
        return
    log_errs(f"Wrong strat index received: {msg.data}")


def recv_position(msg):
    """
    Feedback on /disp/current_position topic.
    """
    if p_dn is None: return  # safety if the function is called before DEC node init

    p_dn.position = [msg.x, msg.y, msg.theta]


def recv_action_done(msg):
    if msg.exit == 1:
        log_info("Last action succeeded.")
        p_dn.nb_actions_done[0] += 1
        return

    if msg.exit ==-1:
        log_errs(f"Last action failed, reason: {msg.reason}")
        return
    log_errs("Wrong value sent on /strat/done_action ...")


def send_action_next(msg):
    """
    Send back the next action when triggered.
    """
    log_info(f"Next action requested by AN")
    p_dn.strategies[p_dn.strat]()
    

#################################################################
#                                                               #
#                         INTERRUPTION                          #
#                                                               #
#################################################################

def park_IT():
    """
    Interrupt : time to park
    """
    log_info('\033[1m\033[36m' + "#"*19 + " Park interrupt " + "#"*18 + '\033[0m')

    next_action_pub.publish(data=[CB_NEXT_ACTION.PARK_IT])


def stop_IT():
    """
    Interrupt : end of match => stop moving
    """
    log_info('\033[1m\033[36m' + "#"*20 + " End of match " + "#"*19 + '\033[0m')

    next_action_pub.publish(data=[CB_NEXT_ACTION.STOP_IT])

#################################################################
#                                                               #
#                           Pubs/Subs                           #
#                                                               #
#################################################################

def init_pubs():
    return
#global next_action_pub
next_action_pub = rospy.Publisher("/strat/next_action_answer", Int16MultiArray, queue_size=10, latch=True)


def init_subs():
    return
#global start_sub, color_sub, strat_sub, position_sub
start_sub = rospy.Subscriber("/game/start", Int16, start_match)
color_sub = rospy.Subscriber("/game/color", Int16, setup_color)
strat_sub = rospy.Subscriber("/game/strat", Int16, setup_strat)
position_sub = rospy.Subscriber("/disp/current_position", Pose2D, recv_position)

#global next_action_sub
next_action_sub = rospy.Subscriber("/strat/next_action_request", Empty, send_action_next)
done_action_sub = rospy.Subscriber("/strat/done_action", EndOfActionMsg, recv_action_done)