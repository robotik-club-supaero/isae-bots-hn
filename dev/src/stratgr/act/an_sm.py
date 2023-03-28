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

import time
import smach

from std_msgs.msg      import Empty
from geometry_msgs.msg import Quaternion

# import SM states defined in an_sm_states package
from an_sm_states.sm_park import Park

from an_const import *
from an_utils import log_info, log_warn, log_errs
from stratgr.act.an_comm  import enable_comm, next_action_pub, next_motion_pub, \
                    stop_teensy_pub

#################################################################
#                                                               #
#                       SM STATE : SETUP                        #
#                                                               #
#################################################################

class Setup(smach.State):
	"""
    STATE MACHINE: setup the SM.
    """

	def __init__(self):
		smach.State.__init__(self, 	outcomes=['start', 'preempted'],
									input_keys=ALL_KEY_LIST,
									output_keys=ALL_KEY_LIST)

	def execute(self, userdata):
		##############################
		## VARIABLES INITIALIZATION ##
		##############################
		
		## Game param variables
		userdata.start = False
		userdata.color = 0
		userdata.score = [SCORES_ACTIONS.INITIAL.value]
		userdata.nb_actions_done = [0]
		
		## Callback of subscribers
		userdata.cb_disp = [-1]  # result of displacement action
		userdata.cb_pos = [[]]  # current position of the robot
		userdata.cb_arm = []    # state of the arm
		userdata.cb_elevator = [] # state of the elevator

		userdata.next_action = -2  # Indicateur de l'action en cours
		userdata.next_pos = Quaternion(x=0, y=0, z=0, w=1)
		userdata.errorReaction = [-1]
		userdata.errorActions = [0]

		## Enable pubs and subs in pr_an_comm.py
		time.sleep(0.01)
		enable_comm()

		##############################
		## WAITING FOR START SIGNAL ##
		##############################
		log_info('[smach] Waiting for "start" signal ...')

		while not userdata.start:
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)
		
		log_info('[smach] starting match !')
		return 'start'

#################################################################
#                                                               #
#                    SM STATE : REPARTITOR                      #
#                                                               #
#################################################################

class Repartitor(smach.State):
	"""
    STATE MACHINE : Dispatch actions between sm substates.
    """

	def __init__(self):
		smach.State.__init__(self, 	outcomes=[e.name for e in LIST_ACTIONS],
									input_keys=['nb_actions_done', 'next_action'],
									output_keys=['nb_actions_done', 'next_action'])

	def execute(self, userdata):
		log_info('[repartitor] requesting next action ...')
		next_action_pub.publish(Empty()) 	         # demande nextAction au DN

		userdata.next_action = LIST_ACTIONS.NONE	     # reset variable prochaine action
		while userdata.next_action == LIST_ACTIONS.NONE:  # en attente de reponse du DN
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)

		userdata.nb_actions_done[0] = 0  		     	  # reinitialisation nb etapes
		return LIST_ACTIONS(userdata.next_action).name    # lancement prochaine action  
		
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class End(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self):
        smach.State.__init__(self, 	outcomes=['end','preempted'],
            						input_keys=[''],
            						output_keys=[''])

    def execute(self, userdata):
        log_info('[end] killing state machine ...')

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        next_motion_pub.publish(Quaternion(x=0,y=0,z=0,w=-1))	# arrêt PF : w = -1
        stop_teensy_pub.publish(Quaternion(x=0,y=0,z=0,w=2))	# arrêt BR (code w=2 pour le BN)
        return 'end'

#################################################################
#                                                               #
#                        INITIALIZATION                         #
#                                                               #
#################################################################

def init_sm(sm):
	"""
	Init state machine with its substates.
	"""

	with sm:
		### Primary States
		smach.StateMachine.add('SETUP', 
			 					Setup(),
								transitions={'preempted':'END','start':'REPARTITOR'})
		smach.StateMachine.add('REPARTITOR', 
			 					Repartitor(),
								transitions={})
		smach.StateMachine.add('END', 
			 					End(),
								transitions={'end':'EXIT_SM','preempted':'EXIT_SM'})

		### Secondary States
		smach.StateMachine.add('PARK', 
			 					Park,
								transitions={'preempted':'REPARTITOR','end':'REPARTITOR'})
