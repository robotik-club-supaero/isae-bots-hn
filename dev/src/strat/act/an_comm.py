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

import os, sys, inspect
import time
import rospy
import smach

from std_msgs.msg      import Int16, Int16MultiArray, Empty, String
from geometry_msgs.msg import Quaternion, Pose2D

from an_const import  DoorCallback, ElevatorCallback, DspCallback, ArmCallback, LoadDetectorCallback, \
                    ClampCallback, COLOR
from an_logging import log_info, log_warn, log_errs

from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import ACTIONS_LIST, Action

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
	p_smData.start = (msg.data == 1)


def setup_color(msg):
    """
    Callback function from topic /sm/color.
    """
    if not ok_comm : return
    if msg.data not in [0,1]:
        log_errs(f"Wrong value of color given ({msg.data})...")
        return
    else: 
        p_smData.color = msg.data
        log_info("Received color : {}".format(COLOR[p_smData.color]))


def cb_next_action(msg):
    """
    Callback for next action (DN -> Repartitor)
    """
    if msg.data[0] == -2:
        # Tmp fix from last year (= sm did not quit normally on parking...)
        p_sm.request_preempt()
        log_info("Received stop signal (initiate parking)")
        return
    if msg.data[0] == -1:
        p_sm.request_preempt()
        log_info("Received park signal (end of match)")
        return
    if msg.data[0] not in range(len(ACTIONS_LIST)): # Index de ACTIONS_LIST dans an_const
        log_errs(f"Wrong command from DN [/strat/repartitor] : {msg.data[0]}")
        return
    p_smData.next_action = [Action(msg.data[0])] + list(msg.data[1:])


def cb_depl_fct(msg):
    """
    Callback of displacement result from Disp Node.
    """
    if not ok_comm: return
    p_smData.cb_depl[0] = DspCallback.parse(msg.data)
    
def cb_position_fct(msg):
    """
    Callback of current position of the robot.
    """
    if not ok_comm: return
    p_smData.robot_pos[0] = msg


def cb_doors_fct(msg):
    """
    Callback of the state of the doors (opened or closed)
    """
    p_smData.cb_doors[0] = DoorCallback.parse(msg.data)

def cb_elevator_fct(msg):
    """
    Callback of the state of the elevator (for the cakes)
    """
    if not ok_comm: return
    p_smData.cb_elevator[0] = ElevatorCallback.parse(msg.data)

def cb_left_arm_fct(msg):
    if not ok_comm: return
    p_smData.cb_left_arm[0] = ArmCallback.parse(msg.data)

def cb_right_arm_fct(msg):
    if not ok_comm: return
    p_smData.cb_right_arm[0] = ArmCallback.parse(msg.data)

def cb_clamp_fct(msg):
    if not ok_comm: return
    p_smData.cb_clamp[0] = ClampCallback.parse(msg.data)

def cb_load_detector(msg):
    if not ok_comm: return
    p_smData.cb_load_detector = LoadDetectorCallback.parse(msg.data)


def cb_park_fct(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_smData.park[0] = msg.data

def get_pickup_id(what, userdata):
    try:
        return userdata.next_action[1]
    except IndexError:
        log_errs(f"No {what} id in userdata.next_action, defaulting to {what} id 0")
        return 0

#################################################################
#                                                               #
#                           ERROR                               #
#                                                               #
#################################################################

# Error action return 
def action_error(reasonOfError): 
	
	##################################
	time_out_error = 3
	##################################

	p_smData.error_reaction[0] = -1

	start_time_error = time.time()
	while p_smData.error_reaction[0] == -1 and time.time()-start_time_error < time_out_error:  # attente de reponse du DN
		time.sleep(0.05)

	if p_smData.error_reaction[0] == 1:  # On decide de faire une autre action suite a l'echec
		p_smData.error_reaction[0] = -1
		log_errs("-> REACTION : SKIP")
		return 'done'	# New action

	if p_smData.error_reaction[0] == 0:  # On decide de reessayer l'action
		p_smData.error_reaction[0] = -1
		log_errs("-> REACTION : RETRY")
		return 'redo'	# Repeat action

	if time.time()-start_time_error > time_out_error:  # Le DN n'a pas pu se decider
		log_errs("-> PAS DE REACTION DU DN")
		p_smData.error_reaction[0] = -1
		return 'done'	# Time out


# update de error_reaction
def update_error_reaction(msg):
	p_smData.error_reaction[0] = msg.data
	log_info('error_reaction mis a jour a {}'.format(p_smData.error_reaction[0]))


#################################################################
#                                                               #
#                          Pubs/Subs                            #
#                                                               #
#################################################################

"""
Initialize all publishers of AN
"""
# GENERAL PUBS
global score_pub, repartitor_pub, callback_action_pub, disp_pub, stop_teensy_pub, remove_obs
score_pub = rospy.Publisher('/game/score', Int16, queue_size=10, latch=True)
repartitor_pub = rospy.Publisher('/strat/action/request', Empty, queue_size=10, latch=True)
callback_action_pub = rospy.Publisher('/strat/action/callback', EndOfActionMsg, queue_size=10, latch=True)
disp_pub = rospy.Publisher('/dsp/order/next_move', Quaternion, queue_size=10, latch=True)
stop_teensy_pub = rospy.Publisher('/stop_teensy', Quaternion, queue_size=10, latch=True)
remove_obs = rospy.Publisher('/removeObs', String, queue_size = 10, latch= True)

# SPECIFIC TO CURRENT YEAR
global doors_pub, elevator_pub, left_arm_pub, right_arm_pub, clamp_pub
doors_pub = rospy.Publisher('/act/order/doors', Int16, queue_size = 10, latch= True)
elevator_pub = rospy.Publisher('/act/order/elevator', Int16, queue_size = 10, latch= True)
left_arm_pub = rospy.Publisher('/act/order/left_arm', Int16, queue_size = 10, latch=True)
right_arm_pub = rospy.Publisher('/act/order/right_arm', Int16, queue_size = 10, latch=True)
clamp_pub = rospy.Publisher('/act/order/clamp', Int16, queue_size = 10, latch=True)
deposit_pub = rospy.Publisher('/simu/deposit_end', Empty, queue_size = 10, latch= True) # ONLY USED BY SIMU INTERFACE # TODO: use to compute score as well?

"""
Initialize all subscribers of AN
"""
# GENERAL SUBS
global start_sub, color_sub, position_sub, repartitor_sub, disp_sub
start_sub = rospy.Subscriber('/game/start', Int16, setup_start)
color_sub = rospy.Subscriber('/game/color', Int16, setup_color)
repartitor_sub = rospy.Subscriber('/strat/action/order', Int16MultiArray, cb_next_action)
disp_sub = rospy.Subscriber('/dsp/callback/next_move', Int16, cb_depl_fct)
position_sub = rospy.Subscriber('/current_position', Pose2D, cb_position_fct)
park_sub = rospy.Subscriber('/park', Int16, cb_park_fct)

# SPECIFIC TO CURRENT YEAR
doors_sub = rospy.Subscriber('/act/callback/doors', Int16, cb_doors_fct)
elevator_sub = rospy.Subscriber('/act/callback/elevator', Int16, cb_elevator_fct)
left_arm_sub = rospy.Subscriber('/act/callback/left_arm', Int16, cb_left_arm_fct)
right_arm_sub = rospy.Subscriber('/act/callback/right_arm', Int16, cb_right_arm_fct)
clamp_sub = rospy.Subscriber('/act/callback/clamp', Int16, cb_clamp_fct)

load_detector = rospy.Subscriber('/act/callback/load_detector', Int16, cb_load_detector) # TODO