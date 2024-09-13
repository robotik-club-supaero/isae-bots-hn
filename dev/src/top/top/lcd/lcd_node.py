#!/usr/bin/env python
# pyright: reportMissingImports=false


import os
import sys
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import lcd_lib
from subprocess   import Popen, PIPE
from std_msgs.msg import Int16


displayFrequency = 10

shortBlinkTime = 0.1
longBlinkTime = 0.4


nbCharsToDisplay = 10

class PointsNode(Node):

	blinkingBackground = True  # choose false to deactivate


	def __init__(self):    
		self.subScore = self.create_subscription(Int16, "/game/score", self.cbScore, QoSProfile())
		self.lcd = lcd_lib.lcd()
		self.lcd.lcd_clear()
		self.lcd.lcd_display_string("SCORE : 0", line=1)


		self.get_logger().info("LCD Node Initialized")


	def cbScore(self, msg):
		self.lcd.lcd_clear()
		self.lcd.lcd_display_string("SCORE : " + str(msg.data), line=1)



### LAUNCH ###

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    
    node = PointsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()
