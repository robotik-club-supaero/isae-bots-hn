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

import os
import sys
import rospy
import signal
import time

from dn_utils    import CONFIG_READER, log_errs, log_fatal, log_info, log_warn, \
                      STRAT_NAMES, DN_LIST_ACTION_NAMES
from dn_msgs     import init_msgs
from dn_strats   import init_strats, \
                      homologation_strat, tests_strat, match_strat

from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

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
        self.strat = int(CONFIG_READER.get("STRAT", "strat_choice"))
        self.strategies = [e.value for e in STRAT_NAMES]

        self.start_time = 0
        self.match_time = int(CONFIG_READER.get("STRAT", "match_time"))
        self.delay_park = 13  # TODO: change it to named constant

        self.actions_ls = [e.value for e in DN_LIST_ACTION_NAMES]
        self.actions_nb = len(self.actions_ls)
        self.curr_action = []
        self.nb_actions_done = [0]

        self.position = [0,0,0]

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

def main():
    rospy.init_node("DEC")

    time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

    node = DecisionsNode()

    init_msgs(node)
    init_strats(node)

    log_info("Waiting for match to start")

    rospy.spin()


if __name__ == "__main__":
    main()