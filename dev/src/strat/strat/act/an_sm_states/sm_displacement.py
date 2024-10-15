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
import yasmin
import math
from enum import IntEnum
from numpy.linalg import norm

from ..an_const import DspCallback, DspOrderMode
from ..an_utils import Sequence
from strat.strat_utils import adapt_pos_to_side, create_quaternion

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
    return create_quaternion(x_d, y_d, t_d, w.value)

def colored_approach(userdata, xd, yd, margin, phase, theta_final=None):
    x, y, _ = adapt_pos_to_side(userdata["robot_pos"].x, userdata["robot_pos"].y, 0, userdata["color"])
    d = norm([xd - x, yd - y])
    
    x_dest = xd + phase.value * margin/d*(xd - x)
    y_dest = yd + phase.value * margin/d*(yd - y)
    if theta_final is None:
        theta_dest = math.atan2(yd - y,xd - x)
    else:
        _, _, theta_dest = adapt_pos_to_side(0, 0, theta_final, userdata["color"])
    
    return colored_destination(userdata["color"], x_dest, y_dest, theta_dest, DspOrderMode.AVOIDANCE)

def colored_approach_with_angle(color, xd, yd, td, margin, theta_final=None):
    xd, yd, td = adapt_pos_to_side(xd, yd, td, color)
    if theta_final is None:
        theta_final = td
    else:
        _, _, theta_final = adapt_pos_to_side(0, 0, theta_final)
    return create_quaternion(xd - margin * math.cos(td), yd - margin * math.sin(td), theta_final, DspOrderMode.AVOIDANCE.value)
                
                
#################################################################
#                                                               #
#                     SM_DISPLACEMENT STATE                     #
#                                                               #
#################################################################

class Displacement(yasmin.State):
    """
    STATE MACHINE : Substate Displacement.

    Handle orders given to the displacement node.
    """

    def __init__(self, node):
        super().__init__(outcomes=['success','fail','preempted'])
        self._node = node
        self._logger = node.get_logger()
        
    def execute(self, userdata):
        # Init the callback var of dsp result. CHECK an_const to see details on cb_depl
        userdata["cb_depl"] = DspCallback.PENDING

        retried = False
        dest = userdata["next_move"]
        self._node.debug_print('c*', f"Displacement Request: towards ({dest.x}, {dest.y}, {dest.z}) with w = {dest.w}")
        self._node.disp_pub.publish(dest)

        init_time = time.time()
        while (time.time() - init_time < DISP_TIMEOUT):
            time.sleep(0.01)

            if self.is_canceled():
                return 'preempted'

            if userdata["cb_depl"] == DspCallback.ERROR_ASSERV:
                self._logger.error("Displacement result: error asserv.")
                # --- try and correct if it's a problem of same position order reject
                return 'fail'

            if userdata["cb_depl"] == DspCallback.PATH_NOT_FOUND:
                self._logger.error("Displacement result: no path found with PF.")
                return 'fail'

            if userdata["cb_depl"] == DspCallback.PATH_BLOCKED:
                self._logger.error("Displacement result: path blocked.")
                return 'fail'

            if userdata["cb_depl"] == DspCallback.DESTINATION_BLOCKED:
                self._logger.error("Displacement result: destination blocked.")
                return 'fail'

            if userdata["cb_depl"] == DspCallback.SUCCESS:

                # FIXME: this fixes a bug (is it?) when the displacement node sometimes reports a success when the robot is blocked by an obstacle
                if math.sqrt((dest.x - userdata["robot_pos"].x) ** 2 + (dest.y - userdata["robot_pos"].y) ** 2) > ACCURACY_MARGIN:
                    self._logger.error('Displacement result: Too far away from target')
                    if retried:
                        return 'fail'
                    else:
                        retried = True
                        self._logger.info('Retrying displacement')
                        userdata["cb_depl"] = DspCallback.PENDING
                        self._node.disp_pub.publish(dest)
                        continue

                self._logger.info('Displacement result: success displacement')
                return 'success'

            
        self._logger.error('Timeout reached - [displacement]')
        return 'fail'

class MoveTo(Sequence):
    def __init__(self, node, destination):
        super().__init__(states=[('COMPUTE_DEST', destination), ('DEPL', Displacement(node))])


class MoveBackwardsStraight(yasmin.State):

    def __init__(self, node, dist):
        super().__init__(outcomes=['fail','success','preempted'])
        self._dist = dist
        self._node = node
        self._logger = node.get_logger()

    def execute(self, userdata):
        
        self._node.debug_print('c', "Request to move backwards")
        
        userdata["cb_depl"] = DspCallback.PENDING

        x,y,theta = userdata["robot_pos"].x, userdata["robot_pos"].y, userdata["robot_pos"].theta
        xd = x - self._dist * math.cos(theta)
        yd = y - self._dist * math.sin(theta)

        self._node.disp_pub.publish(create_quaternion(xd, yd, theta, DspOrderMode.BACKWARDS))
        
        begin = time.perf_counter()
        while time.perf_counter() - begin < DISP_TIMEOUT:           
            time.sleep(0.01)

            if self.is_canceled():
                return 'preempted'

            if userdata["cb_depl"] == DspCallback.ERROR_ASSERV:
                self._logger.error("Displacement result: error asserv.")
                return 'fail'

            if userdata["cb_depl"] == DspCallback.SUCCESS:
                self._logger.info('Displacement result: success displacement')
                return 'success'

        # timeout
        self._logger.warning("Timeout")
        
        return 'fail'
    