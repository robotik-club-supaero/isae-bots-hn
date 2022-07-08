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
#																#
# 							IMPORTS 							#
#																#
#################################################################

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import smach
import time

from geometry_msgs.msg import Quaternion

from pr_an_comm import actionError
from pr_an_comm import disp_pub

from pr_an_utils import LOG_INFO, LOG_ERRS
from pr_an_const import HOME, AWAY, DISPLACEMENT


#################################################################
#																#
# 							DEFINITIONS 						#
#																#
#################################################################

#################################################################
# CONSTANTS
#################################################################

MOVE_TIMEOUT = 30       # Timeout before abandoning move action
MOVE_TIMEOUT_STOP = 9 	# Timeout before changing path 
MOVE_TIMEOUT_DEST = 5   # Timeout before changing action (destination blocked)

STANDARD = 0 
AVOIDING = 6

#################################################################
# FUNCTIONS
#################################################################

def setDest(userdata, x_d, y_d, t_d, w):
	"""Allows a quick conversion of destination given the side played."""
	if not userdata.color in [HOME, AWAY]:
		raise ValueError
	if userdata.color == HOME:  
		userdata.next_pos = Quaternion(x_d, y_d, t_d, w)        	
	if userdata.color == AWAY:
		userdata.next_pos = Quaternion(x_d, 3000-y_d, -t_d, w)


#################################################################
# CLASS DEFINITION
#################################################################

class Displacement(smach.State): # Se deplace vers prochain_endroit
	
	"""Provides the DISPLACEMENT substate."""

	def __init__(self):
		smach.State.__init__(	self, 
								outcomes=['preempted','done','redo','fail'],
								input_keys=['cb_disp','nbActionsDone','next_pos','cb_pos','color'],
								output_keys=['cb_disp','nbActionsDone','next_pos','cb_pos'])

	def execute(self, userdata):
		####################################
		dest = userdata.next_pos
		LOG_INFO("Displacement request to ({}, {}, {}) with w = {}".format(dest.x, dest.y, dest.z, dest.w))
		disp_pub.publish(dest)	# envoie du prochain endroit au PF

		####################################
		userdata.cb_disp[0] = 10  # initialisation du callback 
		begin_time = time.time()

		####################################
		while (time.time() - begin_time < MOVE_TIMEOUT) :

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)

			if userdata.cb_disp[0] == -2:		# Erreur d'asserv | TODO manage same target position rejected...
				LOG_ERRS("Error asserv.")
				# --- try and correct if it's a problem of same position order reject
				x0, y0, _ = userdata.cb_pos[0]
				x1, y1, _ = dest.x, dest.y, _
				if abs(x0-x1) < 5 and abs(y0-y1) < 5:
					LOG_ERRS("Error asserv decoded: same point -> order rejected.")
					setDest(userdata, dest.x, dest.y, dest.z, DISPLACEMENT['rotation'])
					return "redo"	
				return "fail"

			if userdata.cb_disp[0] == -1:		# Aucun path trouve
				LOG_ERRS("No path found with PF.")
				return 'fail'
				#return actionError("noPath")

			if userdata.cb_disp[0] == 0 :		# deplacement reussi
				LOG_INFO('Success displacement')
				userdata.nbActionsDone[0] += 1
				return 'done'

			if userdata.cb_disp[0] == 1 :		# adversaire bloquant
				# Wait till no more obstacle
				begin_stop = time.time()
				while userdata.cb_disp[0] != 2 and time.time()-begin_stop < MOVE_TIMEOUT_STOP:
					time.sleep(0.01)
					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'

				if userdata.cb_disp[0] == 2:	# on est repartis et on attend
					LOG_INFO('Resume displacement')
					userdata.cb_disp[0] = 10
					# setDest(userdata, dest.x, dest.y, dest.z, STANDARD)
					return 'redo'
				elif userdata.cb_disp[0] == 4:
					LOG_INFO('Reduce speed to final light')
					userdata.cb_disp[0] = 10
					# setDest(userdata, dest.x, dest.y, dest.z, DISPLACEMENT["accur_av"])
					return 'redo'
				# else : 							# sinon on est encore bloques, changement de path
				# 	LOG_INFO('Failed displacement - try again with avoidance')
				# 	setDest(userdata, dest.x, dest.y, dest.z, AVOIDING)
				# 	return 'redo'
				return 'fail'
			
			if userdata.cb_disp[0] == 3:		# destination bloquee
				# Wait timeout
				begin_stop = time.time()
				while userdata.cb_disp[0] != 2 and time.time()-begin_stop < MOVE_TIMEOUT_DEST:
					time.sleep(0.01)
					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'
										# sinon on est encore bloques, changement de path
				# 	LOG_INFO('Failed displacement - try again with avoidance')
				# 	setDest(userdata, dest.x, dest.y, dest.z, AVOIDING)
				# 	retu
				if userdata.cb_disp[0] == 2:
					LOG_INFO('Resume displacement.')
					userdata.cb_disp[0] = 10
					return 'redo'
				else:
					LOG_INFO('Blocked destination - Skip action.')
					return 'fail'
					#return actionError('noDest')


		if time.time() - begin_time >= MOVE_TIMEOUT :
			LOG_INFO('Timeout reached')
			return actionError('fail')
