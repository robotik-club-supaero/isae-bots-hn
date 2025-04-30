import time
import sys
import re
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

from .button import ButtonState, GpioButton, DummyLed, LedState
from .optional import Optional
from .oled import OledScreen
from .speaker import Speaker
from .log_reader import ProcessLogReader

from config.qos import default_profile

BUTTON_PIN = 15

class Status(IntEnum):
    INVALID = -1
    INACTIVE = 0
    STOPPING = 1
    STARTING = 2
    STARTED = 3
    IN_MATCH = 4

class MasterNode(Node):

    LOG_LINES = 3
    LOG_STRIP_PATTERN = r'(\[[A-Z]{3}\]|\/[A-Z]{3})[:\s]*' 
    LOG_WRAP_LEN = 21

    LED_BLINK_INTERVAL = 0.25 # s

    OLED_CLEAR_TIMEOUT = 2 # s

    def __init__(self):
        super().__init__("master_node")

        logger = self.get_logger()

        logger.info("Initializing Master node ...")

        self._button = GpioButton(BUTTON_PIN)
        self._buttonState = self._button.getButtonState()
        self._led = DummyLed()

        self._oled = Optional(logger, OledScreen)
        self._oled.display_image('SRC_OledLogo2.ppm')

        self._speaker = Optional(logger, Speaker)
        self._launchMatch = None

        self.status = Status.INACTIVE
        self._startTime = None
        self._stopTime = None

        ### Subscriptions ###
        self.start_sub = self.create_subscription(Int16, '/game/start', self.cb_start, default_profile)

        logger.info("Master node initialized")

    def cb_start(self, msg):
        if msg.data == 1 and self.status == Status.STARTED:
            self._speaker.playSound("cestParti")
            self.status = Status.IN_MATCH

    def update_state(self):
        self._led.update()

        logger = self.get_logger()

        newButtonState = self._button.getButtonState()

        if newButtonState != self._buttonState:
            self._buttonState = newButtonState

            if newButtonState == ButtonState.OFF and self.status >= Status.STARTING:
                logger.info("Button OFF")
                self._speaker.playSound('endRos')              
                
                self.status = Status.STOPPING
                self._launchMatch.terminate()
                self._led.setLedBlinking(MasterNode.LED_BLINK_INTERVAL)

            elif newButtonState == ButtonState.ON and self.status <= Status.STOPPING:
                logger.info("Button ON")              
                self._speaker.playSound('startRos')
                
                self.status = Status.STARTING
                self._launchMatch = ProcessLogReader(
                    cmd=["ros2", "launch", "scripts/match.launch", 'BR:="/dev/ttyBR"', 'ACT:="/dev/ttyACT"'],
                    max_log_lines=MasterNode.LOG_LINES
                )
                self._startTime = time.time()
                self._led.setLedBlinking(MasterNode.LED_BLINK_INTERVAL)
                  
            else:
                logger.error(f"ERROR : unknown button callback {self._buttonState}")

        if self.status != Status.INACTIVE and self._launchMatch.poll() is not None:
            if self.status >= Status.STARTING:
                logger.error("Match script has died unexpectedly")
                self._speaker.playSound('error')
            else:
                logger.info("Match script has exited successfully")
                self._speaker.playSound('RosEnded')
                
            self.status = Status.INACTIVE
            self._led.setLedState(LedState.OFF)
            self._stopTime = time.time()

        if self.status == Status.STARTING:
            # TODO: other way to trigger state change?
            if time.time() - self._startTime > 1:
                logger.info("Match script ready")
                self._speaker.playSound('RosReady')
                self.status = Status.STARTED
                self._led.setLedState(LedState.ON)
        
        if self._launchMatch is not None and self._launchMatch.is_dirty:
            logs = self._launchMatch.get_logs()

            transformedLogs = []
            for log in logs:
                # Remove log header (only keep message)
                output_line = re.split(MasterNode.LOG_STRIP_PATTERN, log)[-1]
                transformedLogs.append(output_line[:MasterNode.LOG_WRAP_LEN])
                transformedLogs.append(output_line[MasterNode.LOG_WRAP_LEN:])
 
            self._oled.display_lines(transformedLogs)
        
        if self.status == Status.INACTIVE and self._buttonState == ButtonState.OFF:
            if self._stopTime is not None and time.time() - self._stopTime > MasterNode.OLED_CLEAR_TIMEOUT:
                self._stopTime = None
                self._oled.display_image('SRC_OledLogo2.ppm')
    
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.status = Status.STOPPING
        if self._launchMatch is not None:
            self._launchMatch.__exit__(*args)
                    
    def run(self):
        try:
            while rclpy.ok(): 
                rclpy.spin_once(self, timeout_sec=0.01)
                self.update_state()
        finally:
            self._oled.display_string("-- EXITED --")

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = MasterNode()
    try:
        with node:
            node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()