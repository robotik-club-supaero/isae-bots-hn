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
import rospy
import threading
import numpy as np
from dn_utils import log_info, log_errs, log_warn, TERM_SIZE, COLOR
from std_msgs.msg      import Empty, Int16, Int16MultiArray
from geometry_msgs.msg import Quaternion, Pose2D

from message.msg       import InfoMsg, ActionnersMsg, EndOfActionMsg

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import Action, ActionResult, ActionScore,  PLANTS_POS as PLANTS_POS_RAW, POTS_POS as POTS_POS_RAW, DEPOSIT_POS as DEPOSIT_POS_RAW, PARK_POS as PARK_POS_RAW

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

PLANT_CAPACITY = 6
CULTURE_SLOTS = 12 # per area

PLANTS_POS = np.array(PLANTS_POS_RAW)
POTS_POS = np.array(POTS_POS_RAW)[:, :2]
DEPOSIT_POS = np.array(DEPOSIT_POS_RAW)[:, :2]
PARK_POS = np.array(PARK_POS_RAW)[:, :2]

p_dn = None

def init_comm(dn):
    """
    Init the decision node in this file (no double imports)
    """
    global p_dn
    p_dn = dn

    #init_pubs()

def publishScore():
    score_pub.publish(data=p_dn.score)

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
    if msg.data not in [0,1]:
        log_errs(f"Wrong value of color given ({msg.data})...")
        return
    else: 
        p_dn.color = msg.data
        log_info("Received color : {}".format(COLOR[p_dn.color]))


def setup_strat(msg):
    """
    Feedback on strategy chosen /game/strat.*
    """
    if p_dn is None: return  # safety if the function is called before DEC node init

    if msg.data not in range(len(p_dn.strategies)):
        log_errs(f"Wrong value of strat given ({msg.data})...")
        return
    else:
        p_dn.strat = msg.data
        log_info(f"Received strat: {p_dn.strategies[p_dn.strat]}")


def recv_position(msg):
    """
    Feedback on /disp/current_position topic.
    """
    if p_dn is None: return  # safety if the function is called before DEC node init

    p_dn.position = [msg.x, msg.y, msg.theta]


def recv_action_callback(msg):
    if msg.exit == ActionResult.SUCCESS:

        p_dn.action_successful = True
        p_dn.retry_count = 0
        
        # FIXME: where should this code be?
        # TODO: how to estimate score of "coccinelles"?
        if p_dn.curr_action[0] == Action.TURN_SOLAR_PANEL:
            p_dn.solar_panels[p_dn.curr_action[1]] = True
            p_dn.score += ActionScore.SCORE_SOLAR_PANEL.value
            publishScore()
        
        if p_dn.curr_action[0] == Action.PICKUP_PLANT:
            p_dn.remaining_plants[p_dn.curr_action[1]] -= PLANT_CAPACITY
        
        if p_dn.curr_action[0] == Action.PICKUP_POT:
            p_dn.remaining_pots[p_dn.curr_action[1]] -= PLANT_CAPACITY
        
        if p_dn.curr_action[0] == Action.DEPOSIT_POT:
            p_dn.deposit_slots[p_dn.curr_action[1]] -= PLANT_CAPACITY
            p_dn.score += PLANT_CAPACITY * ActionScore.SCORE_DEPOSIT_PLANTS.value
            publishScore()
        
        if p_dn.curr_action[0] == Action.PARK:
            p_dn.score += ActionScore.SCORE_PARK.value
            publishScore()
            p_dn.parked = True
        
        log_info("Last action succeeded.")
        return

    if msg.exit == ActionResult.NOTHING_TO_PICK_UP:
        log_warn(f"Last action aborted: there was nothing to pick up")
        if p_dn.curr_action[0] == Action.PICKUP_PLANT:
            p_dn.remaining_plants[p_dn.curr_action[1]] = 0
        elif p_dn.curr_action[0] == Action.PICKUP_POT:
            p_dn.remaining_pots[p_dn.curr_action[1]] = 0
        else:
            log_errs(f"Invalid exit value NOTHING_TO_PICK_UP for action {p_dn.curr_action[0]}")
        return

    if msg.exit == ActionResult.FAILURE:
        log_errs(f"Last action failed, reason: {msg.reason}")
        return
    log_errs("Wrong value sent on /strat/done_action ...")


def send_action_next(msg):
    """
    Send back the next action when triggered.
    """
    log_info(f"Next action requested by AN")
    p_dn.strat_functions[p_dn.strat]()
    

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
    p_dn.go_park = True
    park_pub.publish(data=1)


def stop_IT():
    """
    Interrupt : end of match => stop moving
    """
    log_info('\033[1m\033[36m' + "#"*20 + " End of match " + "#"*19 + '\033[0m')

    next_action_pub.publish(data = [Action.END.value])

#################################################################
#                                                               #
#                           Pubs/Subs                           #
#                                                               #
#################################################################

#global start_sub, color_sub, strat_sub, position_sub
start_sub = rospy.Subscriber("/game/start", Int16, start_match)
color_sub = rospy.Subscriber("/game/color", Int16, setup_color)
strat_sub = rospy.Subscriber("/game/strat", Int16, setup_strat)
position_sub = rospy.Subscriber("/current_position", Pose2D, recv_position)

next_action_pub = rospy.Publisher("/strat/action/order", Int16MultiArray, queue_size=10, latch=True)
next_action_sub = rospy.Subscriber("/strat/action/request", Empty, send_action_next)
done_action_sub = rospy.Subscriber("/strat/action/callback", EndOfActionMsg, recv_action_callback)

score_pub = rospy.Publisher('/game/score', Int16, queue_size=10, latch=True)
end_pub = rospy.Publisher('/game/end', Int16, queue_size=10, latch=True)
park_pub = rospy.Publisher('/park', Int16, queue_size=10, latch=True)