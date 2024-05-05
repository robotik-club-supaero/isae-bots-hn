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
from an_logging import log_info, log_errs, log_warn, debug_print
from an_utils import AutoSequence
from strat_utils import adapt_pos_to_side

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

DISP_TIMEOUT = 30       #[s]
ACCURACY_MARGIN = 10 # [mm]

class Approach(IntEnum):
	INITIAL = -1
	FINAL = 1

def colored_destination(color, x_d, y_d, t_d, w):
	"""Allows a quick conversion of destination given the side played."""
	x_d, y_d, t_d = adapt_pos_to_side(x_d, y_d, t_d, color)
	return Quaternion(x_d, y_d, t_d, w.value)

def colored_approach(userdata, xd, yd, margin, phase, theta_final=None):
	x, y, _ = adapt_pos_to_side(userdata.robot_pos[0].x, userdata.robot_pos[0].y, 0, userdata.color)
	d = norm([xd - x, yd - y])
	
	x_dest = xd + phase.value * margin/d*(xd - x)
	y_dest = yd + phase.value * margin/d*(yd - y)
	if theta_final is None:
		theta_dest = math.atan2(yd - y,xd - x)
	else:
		_, _, theta_dest = adapt_pos_to_side(0, 0, theta_final, userdata.color)
	
	return colored_destination(userdata.color, x_dest, y_dest, theta_dest, DspOrderMode.AVOIDANCE)

def colored_approach_with_angle(color, xd, yd, td, margin, theta_final=None):
	xd, yd, td = adapt_pos_to_side(xd, yd, td, color)
	if theta_final is None:
		theta_final = td
	else:
		_, _, theta_final = adapt_pos_to_side(0, 0, theta_final)
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

		retried = False
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

			if userdata.cb_depl[0] == DspCallback.PATH_BLOCKED:
				log_errs("Displacement result: path blocked.")
				return 'fail'

			if userdata.cb_depl[0] == DspCallback.DESTINATION_BLOCKED:
				log_errs("Displacement result: destination blocked.")
				return 'fail'

			if userdata.cb_depl[0] == DspCallback.SUCCESS:

				# FIXME: this fixes a bug (is it?) when the displacement node sometimes reports a success when the robot is blocked by an obstacle
				if math.sqrt((dest.x - userdata.robot_pos[0].x) ** 2 + (dest.y - userdata.robot_pos[0].y) ** 2) > ACCURACY_MARGIN:
					log_info('Displacement result: Too far away from target')
					if retried:
						return 'fail'
					else:
						retried = True
						log_info('Retrying displacement')
						userdata.cb_depl[0] = DspCallback.PENDING
						disp_pub.publish(dest)
						continue

				log_info('Displacement result: success displacement')
				return 'success'

			
		log_errs('Timeout reached - [displacement]')
		return 'fail'

class MoveTo(AutoSequence):
    def __init__(self, destination):
        super().__init__(('COMPUTE_DEST', destination), ('DEPL', Displacement()))    


class MoveBackwardsStraight(smach.State):

    def __init__(self, dist):
        smach.State.__init__(	self,
                                outcomes=['fail','success','preempted'],
                                input_keys=['cb_depl', 'next_action', 'color', 'robot_pos'],
                                output_keys=['cb_depl'])
        self._dist = dist
        
    def execute(self, userdata):
        
        debug_print('c', "Request to move backwards")
        
        userdata.cb_depl[0] = DspCallback.PENDING

        x,y,theta = userdata.robot_pos[0].x, userdata.robot_pos[0].y, userdata.robot_pos[0].theta
        xd = x - self._dist * math.cos(theta)
        yd = y - self._dist * math.sin(theta)

        disp_pub.publish(Quaternion(xd, yd, theta, DspOrderMode.BACKWARDS))
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < DISP_TIMEOUT:           
            time.sleep(0.01)

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if userdata.cb_depl[0] == DspCallback.ERROR_ASSERV:
                log_errs("Displacement result: error asserv.")
                return 'fail'

            if userdata.cb_depl[0] == DspCallback.SUCCESS:
                log_info('Displacement result: success displacement')
                return 'success'

        # timeout
        log_warn("Timeout")
        
        return 'fail'
    