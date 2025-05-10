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

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import sys
import time
import threading

from .dn_const import *
import strat.dec.dn_strats as dn_strats

from std_msgs.msg import Int16, Int16MultiArray, Empty
from message.msg import EndOfActionMsg
from br_messages.msg import Position

from ..strat_const import Action, ActionScore, ActionResult
from config import StratConfig, COLOR
from config.qos import default_profile, latch_profile, br_position_topic_profile

#################################################################
#                                                               #
#                           DEC node                            #
#                                                               #
#################################################################

class DecisionsNode(Node):
    """
    DEC node: ROS node for decisions and strategy.
    """

    def __init__(self):
        super().__init__("DEC")
        self.get_logger().info("Initializing DEC node ...")

        self.start_sub = self.create_subscription(Int16, "/game/start", self.start_match, default_profile)
        self.color_sub = self.create_subscription(Int16, "/game/color", self.setup_color, default_profile)
        self.strat_sub = self.create_subscription(Int16, "/game/strat", self.setup_strat, default_profile)
        self.strat_sub = self.create_subscription(Int16, "/game/init_pos", self.setup_init_pos, default_profile)
        self.position_sub = self.create_subscription(Position, "/br/currentPosition", self.recv_position, br_position_topic_profile)

        self.next_action_pub = self.create_publisher(Int16MultiArray, "/strat/action/order", latch_profile)
        self.next_action_sub = self.create_subscription(Empty, "/strat/action/request", self.send_action_next, default_profile)
        self.done_action_sub = self.create_subscription(EndOfActionMsg, "/strat/action/callback", self.recv_action_callback, default_profile)

        self.score_pub = self.create_publisher(Int16, '/game/score', latch_profile)
        self.end_pub = self.create_publisher(Int16, '/game/end', latch_profile)
        self.park_pub = self.create_publisher(Int16, '/park', latch_profile)

        self.config = None

        self.match_started = False
        self._setColor(0)
        self.score = ActionScore.SCORE_INIT.value

        self.start_time = 0
        self.match_time = self.config.match_time
        self.delay_park = self.config.delay_park
        self.go_park = False
        self.parked = False
        
        self.strat = self.config.default_strat_index
        self.strategies = self.config.strat_names
        
        self.strat_functions = []
        for stratName in self.strategies:
            try:
                self.strat_functions.append(getattr(dn_strats, stratName))
            except AttributeError:
                self.get_logger().fatal(f"Strategy function name {stratName} doesn't have a function")

        self.park_action = False
        self.kill_action = False
        self.curr_action = [Action.PENDING]  # of type Action

        self.last_action = self.curr_action
        self.action_successful = False
        self.retry_count = 0

        self.position = [0,0,0]  # TODO utilser un objet Pose2D

        self.remaining_stands = [1 for _ in range(len(self.config.pickup_stand_pos))]
        self.deposit_slots = [1 for _ in range(len(self.config.deposit_pos))]
        self.nb_actions = 0

        self.get_logger().info("Decision node initialized")

    def publishScore(self):
        score = Int16()
        score.data = self.score
        self.score_pub.publish(score)

    def publishAction(self):
        self.get_logger().info(f"Publish Action : {str(self.curr_action)}")
        self.action_successful = False
        self.retry_count += 1

        action_msg = Int16MultiArray()
        action_msg.data = [self.curr_action[0].value] + self.curr_action[1:]
        self.next_action_pub.publish(action_msg)
    
    def _setColor(self, color):
        self.color = color
        self.config = StratConfig(color)

    #################################################################
    #                                                               #
    #                           FEEDBACK                            #
    #                                                               #
    #################################################################

    def start_match(self, msg):
        """
        Feedback on start signal /game/start
        """

        if self.match_started: return

        if msg.data == 1:
            self.get_logger().info('\033[1m\033[36m' + "#"*20 + " Start match " + "#"*20 + '\033[0m')

            self.match_started = True
            self.start_time = time.time()
            threading.Timer(self.match_time - self.delay_park, self.park_IT).start()
            threading.Timer(self.match_time, self.stop_IT).start()

    def setup_color(self, msg):
        """
        Feedback on color side /game/color.
        """
       
        if msg.data not in [0,1]:
            self.get_logger().error(f"Wrong value of color given ({msg.data})...")
            return
        else: 
            self.color = msg.data
            self.config = StratConfig(self.color)
            self.get_logger().info("Received color : {}".format(COLOR[self.color]))
    

    def setup_strat(self, msg):
        """
        Feedback on strategy chosen /game/strat.*
        """

        if msg.data not in range(len(self.strategies)):
            self.get_logger().error(f"Wrong value of strat given ({msg.data})...")
            return
        else:
            self.strat = msg.data
            self.get_logger().info(f"Received strat: {self.strategies[self.strat]}")


    def setup_init_pos(self, msg):
        """
        Feedback on strategy chosen /game/init_pos.*
        """

        if msg.data not in [0,1,2]:
            self.get_logger().error(f"Wrong value of init pos given ({msg.data})...")
            return
        else:
            self.init_pos = msg.data
            self.get_logger().info(f"Received init pos: {self.init_pos}")



    def recv_position(self, msg):
        """
        Feedback on /disp/current_position topic.
        """

        self.position = [msg.x, msg.y, msg.theta]


    def recv_action_callback(self, msg):
        if msg.exit == ActionResult.SUCCESS:
            
            self.get_logger().info(f"[ dec_node Callback ] Action {str(self.curr_action)} Success.")

            self.action_successful = True
            self.retry_count = 0
            
            # FIXME: where should this code be?
            # TODO: how to estimate score of "coccinelles"?
            if self.curr_action[0] in (Action.PICKUP_STAND_1, Action.PICKUP_STAND_2):
                pass
            
            if self.curr_action[0] == Action.DEPOSIT_STAND:
                self.score += ActionScore.SCORE_DEPOSIT_STAND.value
                self.publishScore()
            
            if self.curr_action[0] == Action.PARK:
                self.score += ActionScore.SCORE_PARK.value
                self.score += ActionScore.SCORE_COCCINELLE.value
                self.publishScore()
                self.parked = True
        elif msg.exit == ActionResult.NOTHING_TO_PICK_UP:
            self.get_logger().warning(f"Last action aborted: there was nothing to pick up")
            if self.curr_action[0] in (Action.PICKUP_STAND_1, Action.PICKUP_STAND_2):
                self.remaining_stands[self.curr_action[1]] = 0
            elif self.curr_action[0] == Action.PICKUP_POT:
                self.remaining_stands[self.curr_action[1]] = 0
            else:
                self.get_logger().error(f"Invalid exit value NOTHING_TO_PICK_UP for action {self.curr_action[0]}")
        elif msg.exit == ActionResult.FAILURE:
            self.get_logger().error(f"Last action ({self.curr_action[0]}) failed, reason: {msg.reason}")
        else:
            self.get_logger().error("Wrong value sent on /strat/done_action ...")


    def send_action_next(self, msg):
        """
        Send back the next action when triggered.
        """
        self.get_logger().info(f"Next action requested by AN")
        self.strat_functions[self.strat](self)
        

    #################################################################
    #                                                               #
    #                         INTERRUPTION                          #
    #                                                               #
    #################################################################
    
    def park_IT(self):
        """
        Interrupt : time to park
        """
        self.get_logger().info('\033[1m\033[36m' + "#"*19 + " Park interrupt " + "#"*18 + '\033[0m')
        self.go_park = True

        msg = Int16()
        msg.data = 1
        self.park_pub.publish(msg)


    def stop_IT(self):
        """
        Interrupt : end of match => stop moving
        """
        self.get_logger().info('\033[1m\033[36m' + "#"*20 + " End of match " + "#"*19 + '\033[0m')

        action_msg = Int16MultiArray()
        action_msg.data = [Action.END.value]
        self.next_action_pub.publish(action_msg)

        self.end_pub.publish(Int16(data=1))

#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)
    
    node = DecisionsNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
