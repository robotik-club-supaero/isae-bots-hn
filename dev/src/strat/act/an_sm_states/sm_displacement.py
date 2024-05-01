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
import sys
import time
import smach
import math
from enum import IntEnum
from numpy.linalg import norm

from geometry_msgs.msg import Quaternion

from an_const import DspCallback, DspOrderMode
from an_comm import disp_pub
from an_utils import log_info, log_errs, log_warn, debug_print
from strat_utils import adapt_pos_to_side

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

DISP_TIMEOUT = 30       #[s]
STOP_PATH_TIMEOUT = 3   #[s]
STOP_DEST_TIMEOUT = 3   #[s]

class Approach(IntEnum):
	INITIAL = -1
	FINAL = 1

def colored_destination(color, x_d, y_d, t_d, w):
	"""Allows a quick conversion of destination given the side played."""
	# x_d, y_d, t_d = adapt_pos_to_side(x_d, y_d, t_d, color)
	return Quaternion(x_d, y_d, t_d, w.value)

def colored_approach(color, x, y, xd, yd, margin, phase):	   
	#x, y, _ = adapt_pos_to_side(x, y, 0, color)
	d = norm([xd - x, yd - y])
	
	x_dest = xd + phase.value * margin/d*(xd - x)
	y_dest = yd + phase.value * margin/d*(yd - y)
	theta_dest = math.atan2(yd - y,xd - x)
	
	return colored_destination(color, x_dest, y_dest, theta_dest, DspOrderMode.AVOIDANCE)

def colored_approach_with_angle(color, xd, yd, td, margin, theta_final=None):
	if theta_final is None: theta_final = td
	# xd, yd, td = adapt_pos_to_side(xd, yd, td, color)
	return Quaternion(xd - margin * math.cos(td), yd - margin * math.sin(td), theta_final, DspOrderMode.AVOIDANCE.value)
                
                
#################################################################
#                                                               #
#                     SM_DISPLACEMENT STATE                     #
#                                                               #
#################################################################

class Displacement(smach.State):
	"""
	STATE MACHINE : Substate Displacement.

	Handle orders given to the displacement node.
	"""

	def __init__(self):
		smach.State.__init__(	self, 	
		       					outcomes=['success','fail','preempted'],
								input_keys=['cb_depl','robot_pos','next_move','color'],
								output_keys=['cb_depl'])
		
	def execute(self, userdata):
		# Init the callback var of dsp result. CHECK an_const to see details on cb_depl
		userdata.cb_depl[0] = DspCallback.PENDING

		dest = userdata.next_move
		debug_print('c*', f"Displacement Request: towards ({dest.x}, {dest.y}, {dest.z}) with w = {dest.w}")
		disp_pub.publish(dest)

		init_time = time.time()
		while (time.time() - init_time < DISP_TIMEOUT):
			time.sleep(0.01)

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			if userdata.cb_depl[0] == DspCallback.ERROR_ASSERV:
				log_errs("Displacement result: error asserv.")
				# --- try and correct if it's a problem of same position order reject
				return 'fail'

			if userdata.cb_depl[0] == DspCallback.PATH_NOT_FOUND:
				log_errs("Displacement result: no path found with PF.")
				return 'fail'

			if userdata.cb_depl[0] == DspCallback.SUCCESS:
				log_info('Displacement result: success displacement')
				return 'success'

			if userdata.cb_depl[0] == DspCallback.PATH_BLOCKED:
				stop_time = time.time()
				while userdata.cb_depl[0] != DspCallback.RESTART and time.time()-stop_time < STOP_PATH_TIMEOUT:
					time.sleep(0.01)
					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'

				# Once out of the waiting loop
				if userdata.cb_depl[0] == DspCallback.RESTART:	# on est repartis et on attend
					log_info('Displacement restart ...')
					userdata.cb_depl[0] = DspCallback.PENDING
					return 'fail' #NOTE redo
				# Else, we are still blocked...
				return 'fail'

			if userdata.cb_depl[0] == DspCallback.DESTINATION_BLOCKED:
				log_info("RECHERCHE DE CHEMIN")
				userdata.cb_depl[0] = DspCallback.PENDING
				disp_pub.publish(dest)
				stop_time = time.time()
				
				while userdata.cb_depl[0] != DspCallback.RESTART and time.time()-stop_time < STOP_DEST_TIMEOUT:
					time.sleep(0.01)

					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'
					
					if userdata.cb_depl[0] == DspCallback.DESTINATION_BLOCKED:
						disp_pub.publish(dest)
										
				if userdata.cb_depl[0] == DspCallback.RESTART:
					log_info('Displacement restart ...')
					userdata.cb_depl[0] = DspCallback.PENDING
					return 'fail' #NOTE redo
			
				log_warn('Displacement result: dest is blocked.')
				return 'fail'

		if time.time() - init_time >= DISP_TIMEOUT :
			log_errs('Timeout reached - [displacement]')
			return 'fail'
			# return actionError('fail')