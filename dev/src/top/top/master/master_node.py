import time
time.sleep(0.5) # FIXME The program crashes if it starts too soon after the container (why?)
import sys
import re
import subprocess
from enum import IntEnum

import serial  # pyserial — install: pip install pyserial

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Int16
from br_messages.msg import Position

from .button import ButtonState, GpioButton, DummyLed, LedState
from .optional import Optional
from .oled import OledScreen
from .speaker import Speaker
from .log_reader import ProcessLogReader

from config.qos import default_profile, latch_profile

BUTTON_PIN = 15

ACT_DEVICE       = '/dev/ttyACT'
BR_DEVICE        = '/dev/ttyBR'
WATCHDOG_TIMEOUT = 5.0  # seconds without /act/callback_color before Teensy restart + micro_ros connection reset

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

        #self._speaker = Optional(logger, Speaker)
        self._launchMatch = None

        self.status = Status.INACTIVE
        self._startTime = None
        self._stopTime = None

        ### Subscriptions ###
        self.start_sub = self.create_subscription(Int16, '/game/start', self.cb_start, default_profile)

        # --- ACT Teensy watchdog & connection management ---
        # Spawn the ACT micro-ROS agent as a managed subprocess (so we can restart it)
        self._act_agent = self._spawn_act_agent()

        # Track the expected color (set by whoever publishes /game/color)
        self._expected_color = 0  # 0=yellow (default)

        # /game/color: subscribe to know expected color; re-publish if Teensy has the wrong one
        self._color_sub = self.create_subscription(Int16, '/game/color', self._cb_game_color, latch_profile)
        self._color_pub = self.create_publisher(Int16, '/game/color', latch_profile)

        # /game/timer: publish 0 at 1 Hz before match; dec_node takes over during match
        self._game_timer_pub  = self.create_publisher(Int16, '/game/timer', default_profile)
        self._game_timer_tick = self.create_timer(1.0, self._publish_game_timer)

        # /act/callback_color: ping-back from ACT Teensy — echoes its stored color
        self._callback_color_sub = self.create_subscription(
            Int16, '/act/callback_color', self._cb_callback_color, default_profile)

        # Watchdog: 5 s without a callback_color → reset the Teensy.
        # Starts cancelled; activated on first successful callback.
        self._watchdog_armed  = False
        self._watchdog_timer  = self.create_timer(WATCHDOG_TIMEOUT, self._on_watchdog_timeout)
        self._watchdog_timer.cancel() # Wait for first msg

        # ---

        # --- BR Teensy watchdog & connection management ---
        self._br_agent = self._spawn_br_agent()

        self._br_watchdog_armed = False
        self._br_watchdog_timer = self.create_timer(WATCHDOG_TIMEOUT, self._on_br_watchdog_timeout)
        self._br_watchdog_timer.cancel()

        self._br_callback_sub = self.create_subscription(
            Position, '/br/currentPosition', self._cb_br_current_position, default_profile)
        # ---

        self.update_timer = self.create_timer(0.01, self.update_state)

        logger.info("Master node initialized")

    # ------------------------------------------------------------------
    # ACT Teensy watchdog helpers
    # ------------------------------------------------------------------

    def _spawn_act_agent(self):
        return subprocess.Popen(
            ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', ACT_DEVICE],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _cb_game_color(self, msg):
        self._expected_color = msg.data

    def _cb_callback_color(self, msg):
        """Ping-back from ACT Teensy. Resets watchdog and verifies color."""
        if not self._watchdog_armed:
            # First callback after startup or after a reset: activate the watchdog
            self._watchdog_armed = True
            self.destroy_timer(self._watchdog_timer)
            self._watchdog_timer = self.create_timer(WATCHDOG_TIMEOUT, self._on_watchdog_timeout)
        else:
            self._watchdog_timer.reset()

        if msg.data != self._expected_color:
            self.get_logger().warning(
                f"ACT Teensy color mismatch: got {msg.data}, expected {self._expected_color} — resending /game/color"
            )
            resend = Int16()
            resend.data = self._expected_color
            self._color_pub.publish(resend)

    def _publish_game_timer(self):
        """Publish /game/timer = 0 at 1 Hz before match. dec_node takes over once match starts."""
        if self.status == Status.IN_MATCH:
            return
        msg = Int16()
        msg.data = 0
        self._game_timer_pub.publish(msg)

    def _on_watchdog_timeout(self):
        """5 s elapsed without a callback_color from ACT Teensy — reset the connection."""
        self._watchdog_armed = False
        self.destroy_timer(self._watchdog_timer)
        self._watchdog_timer = self.create_timer(WATCHDOG_TIMEOUT, self._on_watchdog_timeout)
        self._watchdog_timer.cancel()

        self.get_logger().error("ACT Teensy watchdog timeout (5 s) — resetting connection")
        #self._speaker.playSound('error')
        self._reset_act_connection()

    def _reset_act_connection(self):
        logger = self.get_logger()

        # 1. Kill the agent subprocess so the serial port is freed
        if self._act_agent and self._act_agent.poll() is None:
            self._act_agent.terminate()
            try:
                self._act_agent.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._act_agent.kill()

        # 2. Toggle DTR to hardware-reset the Teensy (now that the port is free)
        try:
            with serial.Serial(ACT_DEVICE, baudrate=115200, timeout=1) as s:
                s.dtr = False
                time.sleep(0.2)
                s.dtr = True
            logger.info("ACT Teensy reset done")
        except Exception as e:
            logger.error(f"ACT Teensy reset failed: {e}")

        # 3. Wait for the Teensy to finish booting, then restart the agent
        time.sleep(1.5)
        self._act_agent = self._spawn_act_agent()
        logger.info("ACT micro-ROS agent restarted")

    # ------------------------------------------------------------------
    # BR Teensy watchdog helpers
    # ------------------------------------------------------------------

    def _spawn_br_agent(self):
        return subprocess.Popen(
            ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', BR_DEVICE],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _cb_br_current_position(self, msg):
        if not self._br_watchdog_armed:
            self._br_watchdog_armed = True
            self.destroy_timer(self._br_watchdog_timer)
            self._br_watchdog_timer = self.create_timer(WATCHDOG_TIMEOUT, self._on_br_watchdog_timeout)
        else:
            self.get_logger().info(f"RECEIVED BR CALLBACK : {msg.data}")
            self._br_watchdog_timer.reset()

    def _on_br_watchdog_timeout(self):
        self._br_watchdog_armed = False
        self.destroy_timer(self._br_watchdog_timer)
        self._br_watchdog_timer = self.create_timer(WATCHDOG_TIMEOUT, self._on_br_watchdog_timeout)
        self._br_watchdog_timer.cancel()

        self.get_logger().error("BR Teensy watchdog timeout (5 s) — resetting connection")
        self._reset_br_connection()

    def _reset_br_connection(self):
        logger = self.get_logger()

        if self._br_agent and self._br_agent.poll() is None:
            self._br_agent.terminate()
            try:
                self._br_agent.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._br_agent.kill()

        try:
            with serial.Serial(BR_DEVICE, baudrate=115200, timeout=1) as s:
                s.dtr = False
                time.sleep(0.2)
                s.dtr = True
            logger.info("BR Teensy reset done")
        except Exception as e:
            logger.error(f"BR Teensy reset failed: {e}")

        time.sleep(1.5)
        self._br_agent = self._spawn_br_agent()
        logger.info("BR micro-ROS agent restarted")

    # ------------------------------------------------------------------

    def cb_start(self, msg):
        if msg.data == 1 and self.status == Status.STARTED:
            #self._speaker.playSound("cestParti")
            self.status = Status.IN_MATCH

    def update_state(self):
        self._led.update()

        logger = self.get_logger()

        newButtonState = self._button.getButtonState()

        if newButtonState != self._buttonState:
            self._buttonState = newButtonState

            if newButtonState == ButtonState.OFF and self.status >= Status.STARTING:
                logger.info("Button OFF")
                #self._speaker.playSound('endRos')              
                
                self.status = Status.STOPPING
                self._launchMatch.terminate()
                self._led.setLedBlinking(MasterNode.LED_BLINK_INTERVAL)

            elif newButtonState == ButtonState.ON and self.status <= Status.STOPPING:
                logger.info("Button ON")              
                #self._speaker.playSound('startRos')
                
                self.status = Status.STARTING
                self._launchMatch = ProcessLogReader(
                    cmd=["ros2", "launch", "scripts/match.launch", 'BR:="/dev/ttyBR"', 'ACT:="/dev/ttyACT"'],
                    max_log_lines=MasterNode.LOG_LINES,
                    transform=MasterNode._transform_logs,
                    dup="/tmp/match.log"
                )
                self._startTime = time.time()
                self._led.setLedBlinking(MasterNode.LED_BLINK_INTERVAL)
                  
            else:
                logger.error(f"ERROR : unknown button callback {self._buttonState}")

        if self.status != Status.INACTIVE and self._launchMatch.poll() is not None:
            if self.status >= Status.STARTING:
                logger.error("Match script has died unexpectedly")
                #self._speaker.playSound('error')
            else:
                logger.info("Match script has exited successfully")
                #self._speaker.playSound('RosEnded')
                
            self.status = Status.INACTIVE
            self._led.setLedState(LedState.OFF)
            self._stopTime = time.time()

        if self.status == Status.STARTING:
            # TODO: other way to trigger state change?
            if time.time() - self._startTime > 1:
                logger.info("Match script ready")
                #self._speaker.playSound('RosReady')
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
        if self._act_agent and self._act_agent.poll() is None:
            self._act_agent.terminate()
        if self._br_agent and self._br_agent.poll() is None:
            self._br_agent.terminate()
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