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

from message.msg import DisplacementRequest

from ..an_const import DspCallback
from ..an_utils import Sequence

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

DISP_TIMEOUT = 30    #[s]
ACCURACY_MARGIN = 10 # [mm]

class Approach(IntEnum):
    INITIAL = -1
    FINAL = 1

def approach(robot_pos, xd, yd, margin, phase=Approach.INITIAL, theta_final=None, backward=False):
    d = norm([xd - robot_pos.x, yd - robot_pos.y])
    
    x_dest = xd + phase.value * margin/d*(xd - robot_pos.x)
    y_dest = yd + phase.value * margin/d*(yd - robot_pos.y)

    return create_displacement_request(x_dest, y_dest, theta_final, backward)

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
        self._node.disp_pub.publish(dest)

        init_time = time.time()
        while (time.time() - init_time < DISP_TIMEOUT):
            time.sleep(0.01)

            if self.is_canceled():
                return 'preempted'

            if userdata["cb_depl"] == DspCallback.ERROR_ASSERV:
                self._logger.error("Displacement result: error asserv.")
                return 'fail'

            if userdata["cb_depl"] == DspCallback.PATH_NOT_FOUND:
                self._logger.error("Displacement result: no path found with PF.")
                return 'fail'

            if userdata["cb_depl"] == DspCallback.NOT_RECOGNIZED:
                self._logger.error("Displacement result: order rejected because not recognized.")
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
        super().__init__(states=[('COMPUTE_DEST', destination), 
                                 ('DEPL', Displacement(node))])

class MoveStraight(Displacement):
    def __init__(self, node, distance, backward):
        super().__init__(node)
        self.distance = distance
        self.backward = backward

    def execute(self, userdata):
        x,y,theta = userdata["robot_pos"].x, userdata["robot_pos"].y, userdata["robot_pos"].theta

        offset_x = self.distance * math.cos(theta)
        offset_y = self.distance * math.sin(theta)

        if self.backward:
            offset_x = -offset_x
            offset_y = -offset_y

        userdata["next_move"] = create_displacement_request(x + offset_x, y + offset_y, theta, self.backward)

        return super().execute(userdata)

class MoveForwardStraight(MoveStraight):
    def __init__(self, node, distance):
        super().__init__(node, distance, backward=False)

class MoveBackwardsStraight(MoveStraight):
    def __init__(self, node, distance):
        super().__init__(node, distance, backward=True)

def create_displacement_request(x, y, theta=None, backward=False):
    msg = DisplacementRequest()
    msg.kind |= DisplacementRequest.MOVE
    msg.x = float(x)
    msg.y = float(y)

    if theta is not None:
        msg.kind |= DisplacementRequest.ORIENTATION
        msg.theta = float(theta)
    if backward:
        msg.kind |= DisplacementRequest.BACKWARD_FLAG

    return msg

def create_orientation_request(theta):
    msg = DisplacementRequest()
    msg.kind = DisplacementRequest.ORIENTATION
    msg.theta = float(theta)

    return msg

def create_stop_BR_request():
    return DisplacementRequest() # Empty msg = STOP