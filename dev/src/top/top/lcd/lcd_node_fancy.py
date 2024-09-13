#!/usr/bin/env python
# pyright: reportMissingImports=false


import os
import sys
import time

import rclpy
from rclpy.node import Node

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
		super().__init__("points_node")

		self.subScore = self.create_subscription(Int16, "/score", self.cbScore, 10)
		self.lcd = lcd_lib.lcd()
		self.lcd.lcd_clear()
		self.lcd.lcd_display_string("SCORE : 0", line=1)

		self.backlightState = 1
		blink_time = time.time()
		blinkTimer = longBlinkTime

	def run(self):

		self.get_logger().info("Entering Point Node loop")

		# display match logs in case of a real match
		while rclpy.ok():
			rclpy.spin_once(self)

			begin_time = time.time()

			logLine = self.getMatchLog()

			# do things with the log line and display things
			l = len(logLine)
			self.lcd.lcd_display_string(logLine[:min(l, nbCharsToDisplay)], line=2)

			# at some point disable the blinking (when it's ready for example)
			if logLine == "thing": self.blinkingBackground = False


			while time.time() - begin_time < 1/displayFrequency: # to be sure we display at the right frequency if the process takes some time

				if not self.blinkingBackground: self.backlightState == 1  # to make sure the backlight is on when the match starts

				elif time.time() - blink_time > blinkTimer: 

					if self.backlightState == 1: blinkTimer = longBlinkTime
					else: blinkTimer = shortBlinkTime

					blink_time = time.time()
					self.backlightToggle()

				time.sleep(0.001)  # small time resolution

		

	def cbScore(self, msg):
		self.lcd.lcd_clear()  # TODO : should remove the bottom line with the setup logs
		self.lcd.lcd_display_string("SCORE : " + str(msg.data), line=1)
		return



	def getMatchLog(self):

		# this command reads the last line of the matchLog file which is updated by the runMatch.sh script
		with Popen(['tail', '-n', '1', '../../../scripts/matchLog.log'],stdout=PIPE, stderr=PIPE) as process:
			stdout, stderr = process.communicate()

		output = stdout.decode('utf-8')[:-1]

		return output


	def backlightToggle(self):
		'''Changes the state of the background'''

		self.backlightState = 1 - self.backlightState
		self.lcd.backlight(self.backlightState)


	def backLightBlinkOnce(self, blinkTime):
		'''Blink the backlight once during a given amount of time'''

		self.lcd.backlight(0)
		time.sleep(blinkTime)
		self.lcd.backlight(1)



### LAUNCH ###

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    
    node = PointsNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()
