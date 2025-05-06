import time
import sys
import re
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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

    LOG_LINES = 6
    LOG_MATCH_PATTERN = r'\[\w+\]: [\w\W]+'
    LOG_WRAP_LEN = 21

    LED_BLINK_INTERVAL = 0.25 # s

    OLED_CLEAR_TIMEOUT = 1 # s

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

        self.update_timer = self.create_timer(0.01, self.update_state)

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
                    max_log_lines=MasterNode.LOG_LINES,
                    transform=MasterNode._transform_logs
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
            self._oled.display_lines(logs)
        
        if self.status == Status.INACTIVE and self._buttonState == ButtonState.OFF:
            if self._stopTime is not None and time.time() - self._stopTime > MasterNode.OLED_CLEAR_TIMEOUT:
                self._stopTime = None
                self._oled.display_image('SRC_OledLogo2.ppm')

    @staticmethod
    def _transform_logs(line):
        matches = re.findall(MasterNode.LOG_MATCH_PATTERN, line)
        if len(matches) == 0:
            return ()

        output_line = matches[0]
        if len(output_line) > MasterNode.LOG_WRAP_LEN:
            return output_line[:MasterNode.LOG_WRAP_LEN], output_line[MasterNode.LOG_WRAP_LEN:]
        else:
            return output_line, 

    def destroy_node(self):
        self._oled.display_string("-- EXITED --")
        self.status = Status.STOPPING
        if self._launchMatch is not None:
            self._launchMatch.__exit__(exc_type, exc_value, traceback)
      
#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)
    
    node = MasterNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()