#!/usr/bin/env python
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

# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import time
import rospy

from std_msgs.msg      import Int16, Int16MultiArray, Empty
from geometry_msgs.msg import Quaternion, Pose2D

from an_const import *
from an_utils import log_info, log_warn, log_errs, patchFrameBr

#################################################################
if not SIMULATION:
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg 		# sur robot
else:
	from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg	

rospy.set_param("robot_name", "GR")

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

def init_comm(sm):
    """
    Init state machine in this file (no double import possible..)
    """
    global p_sm, p_smData
    p_sm = sm
    p_smData = sm.userdata

    init_pubs()
    init_subs()


def enable_comm():
    """
    Set communication topics open in AN.
    """
    global ok_comm
    ok_comm = True

#################################################################
#                                                               #
#                           FEEDBACK                            #
#                                                               #
#################################################################

global ok_comm
ok_comm = False


def setup_start(msg):
	"""
    Callback function from topic /sm/start.
    """
	p_smData = (msg.data == 1)


def setup_color(msg):
    """
    Callback function from topic /sm/color.
    """
    if msg.data not in [e.value for e in ROBOT_SIDES]:
        log_errs(f"Wrong value of color given ({msg.data})...")
        return
    else: 
        for e in ROBOT_SIDES:
            if e.value == msg.data:
                log_info(f"Color received : {e.name}") 
                p_smData.color = e.value
                break       


def cb_next_action(msg):
    """
    Callback for next action (DN -> Repartitor)
    """
    if msg.data[0] == LIST_ACTIONS.STOP:
        p_sm.request_preempt()
        log_info("Received stop signal (end of match)")
        return
    if msg.data[0] == LIST_ACTIONS.PARK:
        # Tmp fix from last year (= sm did not quit normally on parking...)
        p_sm.request_preempt()
        log_info("Received park signal (almost finished)")
        return
    if msg.data[0] not in [e.value for e in LIST_ACTIONS]:
        log_errs(f"Wrong command from DN [/strat/next_action] : {msg.data[0]}")
        return
    p_smData.next_action = msg.data[0]


def cb_disp(msg):
    """
    Callback of displacement result from Disp Node.
    """
    if not ok_comm: return
    p_smData.cb_disp[0] = msg.data


def cb_position(msg):
    """
    Callback of current position of the robot.
    """
    if not ok_comm: return
    p_smData.cb_pos[0] = [msg.x, msg.y, msg.theta]

def cb_arm(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_smData.cb_arm[0] = msg.data
    
def cb_elevator(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_smData.cb_elevator[0] = msg.data

def cb_XXXXX(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_smData.XXX[0] = msg.data

#################################################################
#                                                               #
#                           ERROR                               #
#                                                               #
#################################################################

# Error action return 
def actionError(reasonOfError): 
	
	##################################
	time_out_error = 3
	##################################

	p_smData.errorReaction[0] = -1

	start_time_error = time.time()
	while p_smData.errorReaction[0] == -1 and time.time()-start_time_error < time_out_error:  # attente de reponse du DN
		time.sleep(0.05)

	if p_smData.errorReaction[0] == 1:  # On decide de faire une autre action suite a l'echec
		p_smData.errorReaction[0] = -1
		log_errs("-> REACTION : SKIP")
		return 'done'	# New action

	if p_smData.errorReaction[0] == 0:  # On decide de reessayer l'action
		p_smData.errorReaction[0] = -1
		log_errs("-> REACTION : RETRY")
		return 'redo'	# Repeat action

	if time.time()-start_time_error > time_out_error:  # Le DN n'a pas pu se decider
		log_errs("-> PAS DE REACTION DU DN")
		p_smData.errorReaction[0] = -1
		return 'done'	# Time out


# update de errorReaction
def updateErrorReaction(msg):
	p_smData.errorReaction[0] = msg.data
	log_info('errorReaction mis a jour a {}'.format(p_smData.errorReaction[0]))

#################################################################
#                                                               #
#                           REQUESTS                            #
#                                                               #
#################################################################

def send_added_score(pts):
    """
    Request for setting a new score (prev + new pts added).
    """
    p_smData.score[0] += pts
    score_pub.publish(p_smData.score[0])

#################################################################
#                                                               #
#                          Pubs/Subs                            #
#                                                               #
#################################################################

def init_pubs():
    """
    Initialize all publishers of AN.
    """
    # GENERAL PUBS
    global score_pub, next_action_pub, done_action_pub, next_motion_pub, stop_teensy_pub
    score_pub = rospy.Publisher('/game/score', Int16, queue_size=10, latch=True)
    next_action_pub = rospy.Publisher('/strat/next_action', Empty, queue_size=10, latch=True)
    done_action_pub = rospy.Publisher('/strat/done_action', EndOfActionMsg, queue_size=10, latch=True)
    next_motion_pub = rospy.Publisher('/disp/next_displacement', Quaternion, queue_size=10, latch=True)
    stop_teensy_pub = rospy.Publisher('/stop_teensy', Quaternion, queue_size=10, latch=True)

    # SPECIFIC TO CURRENT YEAR
    global cherries_pub, elevator_pub
    cherries_pub = rospy.Publisher('/strat/cherries', Int16, queue_size=10, latch=True)
    elevator_pub = rospy.Publisher('/strat/elevator', Int16, queue_size=10, latch=True)


def init_subs():
    """
    Initialize all subscribers of AN.
    """
    # GENERAL SUBS
    global start_sub, color_sub, position_sub, next_action_sub, done_motion_sub
    start_sub = rospy.Subscriber('/game/start', Int16, setup_start)
    color_sub = rospy.Subscriber('/game/color', Int16, setup_color)
    next_action_sub = rospy.Subscriber('/strat/next_action', Int16MultiArray, cb_next_action)
    done_motion_sub = rospy.Subscriber('/disp/done_displacement', Int16, cb_disp)
    position_sub = rospy.Subscriber('/disp/current_position', Pose2D, cb_position)

    # SPECIFIC TO CURRENT YEAR
    global cherries_sub, elevator_sub
    cherries_sub = rospy.Subscriber('/strat/cherries_feedback', Int16, cb_arm)
    elevator_sub = rospy.Subscriber('/strat/elevator_feedback', Int16, cb_elevator)


