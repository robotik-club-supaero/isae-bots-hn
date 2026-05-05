#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false
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

import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Int16
from config.qos import default_profile, latch_profile

from .test_config import TEST_ACTIONS, TEST_TIMEOUT


class ActionTestNode(Node):

    def __init__(self):
        super().__init__('ActionTestNode')
        self.get_logger().info("Initializing Action Test Node.")

        # Publishers
        self.pub_cursor     = self.create_publisher(Int16, '/act/order/cursor', latch_profile)
        self.pub_drawbridge = self.create_publisher(Int16, '/act/order/drawbridge', latch_profile)

        # Subscribers
        self.sub_cursor_cb     = self.create_subscription(Int16, '/act/callback/cursor', self._on_cursor_callback, default_profile)
        self.sub_drawbridge_cb = self.create_subscription(Int16, '/act/callback/drawbridge', self._on_drawbridge_callback, default_profile)

        # State
        self.action_queue   = list(TEST_ACTIONS)
        self.current_action = None
        self.waiting        = False

        # Timeout timer, created once and reset/cancelled as needed
        self.timeout_timer = self.create_timer(TEST_TIMEOUT, self._on_timeout)
        self.timeout_timer.cancel()

        self.get_logger().info(f"Action Test Node initialized — {len(self.action_queue)} actions queued.")
        self._send_next_action()

    # ------------------------------------------------------------------

    def _send_next_action(self):
        if not self.action_queue:
            self.get_logger().warning("All actions completed successfully!")
            self.destroy_node()
            rclpy.try_shutdown()
            return

        self.current_action = self.action_queue.pop(0)
        actuator, order, expected_cb = self.current_action
        self.waiting = True

        msg      = Int16()
        msg.data = int(order)

        if actuator == 'cursor':
            self.get_logger().info(f"→ cursor order: {order}  (expecting callback {expected_cb})")
            self.pub_cursor.publish(msg)
        elif actuator == 'drawbridge':
            self.get_logger().info(f"→ drawbridge order: {order}  (expecting callback {expected_cb})")
            self.pub_drawbridge.publish(msg)
        else:
            self.get_logger().error(f"Unknown actuator type: '{actuator}' — skipping.")
            self.waiting = False
            self._send_next_action()
            return

        self.timeout_timer.reset()

    # ------------------------------------------------------------------

    def _on_cursor_callback(self, msg):
        if not self.waiting or self.current_action is None:
            return
        actuator, order, expected_cb = self.current_action
        if actuator != 'cursor':
            return
        if msg.data == int(expected_cb):
            self.get_logger().info(f"✓ cursor callback: {msg.data}")
            self.timeout_timer.cancel()
            self.waiting = False
            self._send_next_action()
        else:
            self.get_logger().warning(f"Unexpected cursor callback: {msg.data} (expected {expected_cb})")

    def _on_drawbridge_callback(self, msg):
        if not self.waiting or self.current_action is None:
            return
        actuator, order, expected_cb = self.current_action
        if actuator != 'drawbridge':
            return
        if msg.data == int(expected_cb):
            self.get_logger().info(f"✓ drawbridge callback: {msg.data}")
            self.timeout_timer.cancel()
            self.waiting = False
            self._send_next_action()
        else:
            self.get_logger().warning(f"Unexpected drawbridge callback: {msg.data} (expected {expected_cb})")

    def _on_timeout(self):
        self.timeout_timer.cancel()
        actuator, order, expected_cb = self.current_action
        self.get_logger().error(f"Timeout (10 s) waiting for {actuator} callback (expected {expected_cb}) — trying next action.")
        self.timeout_timer.cancel()
        self.waiting = False
        self._send_next_action()


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)
    node = ActionTestNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Action Test Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
