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

import yasmin
import math
import time
from std_msgs.msg import Empty

from config import StratConfig

from ..an_const import DspCallback
from ..an_utils import Sequence, Concurrence, DrawbridgeUP, DrawbridgeDOWN, PumpsOFF

from strat.strat_utils import create_end_of_action_msg
from .sm_displacement import MoveTo, MoveBackwardsStraight, Approach, approach, create_displacement_request, DISP_TIMEOUT

from strat.strat_const import ActionResult

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################


class CalcPositionDepositBox(yasmin.State):

    def __init__(self, node):
        super().__init__(outcomes=['fail', 'success', 'preempted'])
        self._node = node

    def execute(self, userdata):
        zone_id = self._node.get_pickup_id("deposit_zones", userdata)
        
        xp, yp, thetap = StratConfig(userdata["color"]).deposit_zones_pos[zone_id]
        userdata["next_move"] = create_displacement_request(xp, yp, theta=thetap, backward=False)
        return 'success'


class ReportDeposit(yasmin.State): # DEPRECATED TODO
    def __init__(self, deposit_pub):
        super().__init__(outcomes=['success'])
        self._deposit_pub = deposit_pub

    def execute(self, userdata):
        self._deposit_pub.publish(Empty())
        return 'success'


class DepositBoxEnd(yasmin.State): # DEPRECATED TODO

    def __init__(self, node):
        super().__init__(outcomes=['fail', 'success', 'preempted'])
    def execute(self, userdata):
        # TODO check that the action was actually successful
        userdata['action_result'] = ActionResult.SUCCESS
        return 'success'

#################################################################
#                                                               #
#                        SM STATE : DEPOSIT_POTS                #
#                                                               #
#################################################################

class _DrawbridgeSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
        ('DEPOSIT_DRAW_BRIDGE_DOWN', DrawbridgeDOWN(node)),
        ('DEPOSIT_PUMPS_TURN_OFF', PumpsOFF(node)),
        ('DEPOSIT_DRAW_BRIDGE_UP', DrawbridgeUP(node)),
        ])

class DepositBoxesSequence(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('DEPOSIT_MOVE_TO_ZONE', MoveTo(node, CalcPositionDepositBox(node))),
            ('DEPOSIT_BOX_SEQUENCE', _DrawbridgeSequence(node)),
            ('DEPOSIT_BOX_END', DepositBoxEnd(node)),
            ])
