#!/usr/bin/env python3
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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os, sys, inspect
import rospy
import signal
import time

from dn_utils    import READER, log_errs, log_fatal, log_info, log_warn
from dn_comm     import init_comm, CULTURE_SLOTS
from dn_strats   import init_strats, test_strat, homologation, match_strat

from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import Action, ActionScore

def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    log_warn("Node forced to terminate ...")
    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()

#################################################################
#                                                               #
#                           DEC node                            #
#                                                               #
#################################################################

class DecisionsNode:
    """
    DEC node: ROS node for decisions and strategy.
    """

    def __init__(self):

        log_info("Initializing DEC node ...")

        self.match_started = False
        self.color = 0
        self.score = ActionScore.SCORE_INIT.value

        self.start_time = 0
        self.match_time = int(READER.get("STRAT", "match_time"))
        self.delay_park =  int(READER.get("STRAT", "delay_park"))
        self.go_park = False
        self.parked = False

        self.strat = int(READER.get("STRAT", "strat_default"))
        self.strategies = [match_strat, homologation, test_strat]

        self.park_action = False
        self.kill_action = False
        self.curr_action = [Action.PENDING]  # of type Action

        self.last_action = self.curr_action
        self.action_successful = False
        self.retry_count = 0

        self.init_zone = int(READER.get("STRAT", "init_zone"))
        self.position = [0,0,0]  # TODO utilser un objet Pose2D

        self.remaining_plants = [6 for _ in range(6)]
        self.remaining_pots = [6 for _ in range(6)]
        self.deposit_slots = [CULTURE_SLOTS for _ in range(3)]
        self.solar_panels = [False for _ in range(6)]


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rospy.init_node("DEC")

    time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

    node = DecisionsNode()

    init_comm(node)
    init_strats(node)

    log_info("Waiting for match to start")

    rospy.spin()


if __name__ == "__main__":
    main()