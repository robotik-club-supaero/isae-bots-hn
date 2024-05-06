#!/usr/bin/env python
# pyright: reportMissingImports=false


import os
import sys
import time
import rospy
import signal
import lcd_lib
from subprocess   import Popen, PIPE
from std_msgs.msg import Int16


displayFrequency = 10

shortBlinkTime = 0.1
longBlinkTime = 0.4


nbCharsToDisplay = 10

_NODENAME_ = "[LCD]"


#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################


def log_info(msg):
    """
    Print logs standard.
    """
    rospy.loginfo(f"{_NODENAME_} {msg}")


def log_warn(msg):
    """
    Print logs warning.
    """
    rospy.logwarn(f"{_NODENAME_} {msg}")


def log_errs(msg):
    """
    Print logs errors.
    """
    rospy.logerr(f"{_NODENAME_} {msg}")


def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    log_warn("Node forced to terminate ...")
    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()




#######################################################################
# LCD NODE
#######################################################################


class LcdNode:

    blinkingBackground = True  # choose false to deactivate
    isNodeInit = False

    def __init__(self):    

        self.subScore = rospy.Subscriber("/game/score", Int16, self.cbScore)
        
        # color and strat display
        self.stratDisplayed = '*' # display when not initialized
        self.colorDisplayed = '****' # display when not initialized
        self.subScore = rospy.Subscriber("/game/color", Int16, self.cbColor)
        self.subScore = rospy.Subscriber("/game/strat", Int16, self.cbStrat)
        
        # to know when match starts
        self.isMatchStarted = False
        self.subStart = rospy.Subscriber("/game/start", Int16, self.cbStart)
        
        self.lcd = lcd_lib.lcd()
        self.lcd.lcd_clear()
        
        self.lcd.lcd_display_string("     READY      ", line=1)
        self.lcd.lcd_display_string(f"STRAT:{self.stratDisplayed}     {self.colorDisplayed}", line=2)

        self.isNodeInit = True

        rospy.loginfo("LCD Node Initialized")


    def cbScore(self, msg):
        if self.isNodeInit:
            self.lcd.lcd_clear()
            self.lcd.lcd_display_string("SCORE : " + str(msg.data), line=1)

    def cbStrat(self, msg):
        if self.isNodeInit:
            self.stratDisplayed = str(msg.data)
            self.lcd.lcd_display_string(f"STRAT:{self.stratDisplayed}     {self.colorDisplayed}", line=2)
        
    def cbColor(self, msg):
        if self.isNodeInit:
            if msg.data == 0:
                self.colorDisplayed = "HOME"
            elif msg.data == 1:
                self.colorDisplayed = "AWAY"
            else:
                log_errs("Color msg is neither 0 or 1")
            self.lcd.lcd_display_string(f"STRAT:{self.stratDisplayed}     {self.colorDisplayed}", line=2)
        
        
    def cbStart(self, msg):
        if msg.data == 1 and not self.isMatchStarted:
            self.isMatchStarted = True
        
            self.lcd.lcd_clear()
            self.lcd.lcd_display_string("SCORE : " + 0, line=1)


### LAUNCH ###

if __name__ == '__main__':
    rospy.init_node('LCD')
    signal.signal(signal.SIGINT, sig_handler)

    node = LcdNode()
    
    rospy.spin()
