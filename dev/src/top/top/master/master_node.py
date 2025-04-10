import time
import subprocess
import sys
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

from .nano import ArduinoCommunicator
from .nano.nanoInterface import ButtonPressState, ButtonColorMode
from .oled import Oled
from .speaker import Speaker

from config.qos import default_profile

class Status(IntEnum):
    INACTIVE = 0
    STOPPING = 1
    STARTING = 2
    STARTED = 3
    IN_MATCH = 4

class MasterNode(Node):
    def __init__(self, nano_port):
        super().__init__("master_node")
        self.declare_parameter('my_parameter', 'world')

        logger = self.get_logger()

        logger.info("Initializing Master node ...")

        self._oled = Oled()
        self._oled.set_bgImage('SRC_OledLogo2.ppm')
        self._speaker = Speaker()

        self._launchMatch = None
        self.status = Status.INACTIVE
        self._startTime = None

        ### Subscriptions ###
        self.start_sub = self.create_subscription(Int16, '/game/start', self.cb_start, default_profile)

        ### Connecting to Nano ###
        logger.debug("Connecting to the nano...")
        while True:
            try:
                self._nanoCom = ArduinoCommunicator(port=nano_port, baudrate=9600)
                if self._nanoCom.establish_communication() == 0:
                    break
                logger.warn("Failed to connect to the nano. Retrying in 2 seconds")
                time.sleep(2)
            except Exception as e:
                logger.warn("Failed to connect to the nano: " + str(e) + ". Retrying in 5 seconds")
                time.sleep(5)
        logger.info("Connected to the nano")
        self.buttonState = self._nanoCom.read_button_state()
        self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_BLINKING, color=(255,0,0))
           
        logger.info("Master node initialized")

    def cb_start(self, msg):
        if msg.data == 1 and self.status == Status.STARTED:
            self._speaker.playSound("cestParti")
            self.status = Status.IN_MATCH

    def update_state(self):
        logger = self.get_logger()

        newButtonState = self._nanoCom.read_button_state()

        if newButtonState != self.buttonState:
            self.buttonState = newButtonState

            if newButtonState == ButtonPressState.BUTTON_PRESS_OFF and self.status >= Status.STARTING:
                logger.info("Button OFF")
                self.speaker.playSound('endRos')
                self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_FADING, color=(255,0,255))
                self._launchMatch.terminate()
                self.status = Status.STOPPING
                                   
            elif self.buttonState == ButtonPressState.BUTTON_PRESS_ON and self.status <= Status.STOPPING:
                logger.info("Button ON")              
                self.speaker.playSound('startRos')
                self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_FADING, color=(255,255,0))                    
               
                self._launchMatch = subprocess.run(
                    ["ros2", "launch", "scripts/match.launch", 'BR:="/dev/ttyBR"', 'ACT:="/dev/ttyACT"', 'LIDAR:="/dev/ttyLIDAR"', 'NANONPX:="/dev/ttyNANO"'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT
                )
                self._startTime = time.time()
                self.status = Status.STARTING
                  
            else:
                logger.error(f"ERROR : unknown button callback {self.buttonState}")

        if self.status == Status.STARTING:
            # TODO: other way to trigger state change?
            if time.time() - self._startTime > 1:
                self.speaker.playSound('RosReady')
                self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_STATIC, color=(0,255,0))
                self.status = Status.STARTED
        
        if self.status == Status.STOPPING:
            if self._launchMatch.poll() is not None:
                self.speaker.playSound('RosEnded')
                self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_STATIC, color=(255,0,0))
                self.status = Status.INACTIVE

    def run(self):
        while rclpy.ok(): 
            rclpy.spin_once(self, timeout_sec=0.01)
            self.update_state()

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = MasterNode('/dev/ttyUSB0') # TODO: use a parameter instead of hard-coded device
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()