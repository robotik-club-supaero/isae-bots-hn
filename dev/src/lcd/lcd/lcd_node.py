#!/usr/bin/env python
# pyright: reportMissingImports=false

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import signal
import lcd_lib
from subprocess   import Popen, PIPE
from std_msgs.msg import Int16


displayFrequency = 10

shortBlinkTime = 0.1
longBlinkTime = 0.4


nbCharsToDisplay = 10

#######################################################################
# LCD NODE
#######################################################################


class LcdNode(Node):

    blinkingBackground = True  # choose false to deactivate
    isNodeInit = False

    def __init__(self): 
        super().__init__("LCD")   

        qos_profile = QoSProfile()
        self.subScore = self.create_subscription(Int16, "/game/score", self.cbScore, qos_profile)
        
        # color and strat display
        self.stratDisplayed = '*' # display when not initialized
        self.colorDisplayed = '****' # display when not initialized
        self.initZoneDisplayed = "*"
        self.subScore = self.create_subscription(Int16, "/game/color", self.cbColor, qos_profile)
        self.subScore = self.create_subscription(Int16, "/game/strat", self.cbStrat, qos_profile)
        self.subInitZone = self.create_subscription(Int16, "/game/init_pos", self.cbInitZone, qos_profile)
        
        # to know when match starts
        self.isMatchStarted = False
        self.subStart = self.create_subscription(Int16, "/game/start", self.cbStart, qos_profile)
        
        self.lcd = lcd_lib.lcd()
        self.lcd.lcd_clear()
        
        self.lcd.lcd_display_string("     READY      ", line=1)
        self.lcd.lcd_display_string(f"STRAT:{self.stratDisplayed}     {self.colorDisplayed}", line=2)

        self.isNodeInit = True

        self.log_info("LCD Node Initialized")

    def cbScore(self, msg):
        if self.isNodeInit:
            self.lcd.lcd_clear()
            self.lcd.lcd_display_string("SCORE : " + str(msg.data), line=1)

    def cbStrat(self, msg):
        if self.isNodeInit:
            self.stratDisplayed = str(msg.data)
            self.lcd.lcd_display_string(f"STRAT:{self.stratDisplayed}     {self.colorDisplayed}", line=2)

    def cbInitZone(self, msg):
        if self.isNodeInit:
            self.initZoneDisplayed = str(msg.data)
            self.lcd.lcd_display_string(f"ZONE:{self.initZoneDisplayed}     {self.colorDisplayed}", line=2)
        
    def cbColor(self, msg):
        if self.isNodeInit:
            if msg.data == 0:
                self.colorDisplayed = "HOME"
            elif msg.data == 1:
                self.colorDisplayed = "AWAY"
            else:
                self.log_errs("Color msg is neither 0 or 1")
            self.lcd.lcd_display_string(f"STRAT:{self.stratDisplayed}     {self.colorDisplayed}", line=2)
        
        
    def cbStart(self, msg):
        if msg.data == 1 and not self.isMatchStarted:
            self.isMatchStarted = True
        
            self.lcd.lcd_clear()
            self.lcd.lcd_display_string("SCORE : " + "0", line=1)

    def log_info(self, msg):
        """
        Print logs standard.
        """
        self.get_logger().info(msg)


    def log_warn(self, msg):
        """
        Print logs warning.
        """
        self.get_logger().warning(msg)


    def log_errs(self, msg):
        """
        Print logs errors.
        """
        self.get_logger().error(msg)


### LAUNCH ###

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    node = LcdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_warn("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()