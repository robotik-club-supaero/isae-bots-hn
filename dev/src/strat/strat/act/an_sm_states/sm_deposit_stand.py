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

from ..an_const import DspOrderMode, DspCallback, R_APPROACH_STAND
from ..an_utils import Sequence, DescendElevator, OpenClamp, CloseClamp, Concurrence

from strat.strat_const import DEPOSIT_POS
from strat.strat_utils import adapt_pos_to_side, create_end_of_action_msg
from .sm_displacement import MoveTo, MoveBackwardsStraight, Approach, colored_approach_with_angle, DISP_TIMEOUT

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################


class CalcPositionningStand(yasmin.State):

    def __init__(self, get_pickup_id):
        super().__init__(outcomes=['fail', 'success', 'preempted'])
        self._get_pickup_id = get_pickup_id

    def execute(self, userdata):
        x, y = userdata["robot_pos"].x, userdata["robot_pos"].y
        pots_id = self._get_pickup_id("deposit pots", userdata)

        xp, yp, thetap = DEPOSIT_POS[pots_id]
        userdata["next_move"] = colored_approach_with_angle(userdata["color"], xp, yp, thetap, R_APPROACH_STAND)

        return 'success'


class ReportDeposit(yasmin.State): # DEPRECATED TODO
    def __init__(self, deposit_pub):
        super().__init__(outcomes=['success'])
        self._deposit_pub = deposit_pub

    def execute(self, userdata):
        self._deposit_pub.publish(Empty())
        return 'success'


class DepositStandEnd(yasmin.State): # DEPRECATED TODO

    def __init__(self, callback_action_pub):
        super().__init__(outcomes=['fail', 'success', 'preempted'])
        self._callback_action_pub = callback_action_pub

    def execute(self, userdata):
        # TODO check that the action was actually successful
        self._callback_action_pub.publish(create_end_of_action_msg(exit=1, reason='success'))

        return 'success'

#################################################################
#                                                               #
#                        SM STATE : DEPOSIT_POTS                 #
#                                                               #
#################################################################


class DepositStand(Sequence):
    def __init__(self, node):
        super().__init__(states=[
            ('DEPL_POSITIONING_DEPOSIT', MoveTo(node, CalcPositionningStand(node.get_pickup_id))),
            ('DESCEND_ELEVATORS', Concurrence(
                    ELEVATOR_1 = DescendElevator(node, 1),
                    ELEVATOR_2 = DescendElevator(node, 2)
            )),
            ('OPEN_CLAMPS', Concurrence(
                    CLAMP_1 = OpenClamp(node, 1),
                    CLAMP_2 = OpenClamp(node, 2)
            )), 
            ('DEPL_MOVEBACK_DEPOSIT', MoveTo(node, MoveBackwardsStraight(node, 200))), # TODO 200 = 20 cm for now
            ('REPORT_TO_INTERFACE', ReportDeposit(node.deposit_pub)),
            ('DEPOSIT_POTS_END',  DepositStandEnd(node.callback_action_pub)),
        ])
