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

import time
import sys
from enum import IntEnum, Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Bool

from .ISBManager import ISBManager, LedState
from config import NaiveStratConfig, COLOR
from config.qos import default_profile, latch_profile

DEVICE_BUS = 1
DEVICE_ADDR = 0x20
DEVICE_NB_GPIOS = 16

BLINK_PERIOD = 0.25 # s

LED_PINS = [15,14,13,12,11,10,9,8,7,6]
BUTTONS_PINS = [0,1,2,3,4]
TRIGGER_PIN = 5

TRIGGER_LED_ID = 0
BR_IDLE_LED_ID = 1
STATUS_LED_ID = len(LED_PINS) - 1

STRAT_BUTTON_ID = 0
COLOR_BUTTON_ID = 1
INIT_POS_BUTTON_ID = 2
BR_IDLE_BUTTON_ID = 3

class BR_Callback(IntEnum):
    OK_READY = 5
    OK_IDLE = 6
    # Non exhaustive

class ISBNode(Node):
    def __init__(self):
        super().__init__("ISB")

        config = NaiveStratConfig()

        self.strat_count = len(config.strat_names)
        self.init_pos_count = config.init_zone_count

        self.current_strat = Int16(data=config.default_strat_index)
        self.current_color = Int16(data=0)
        self.current_init_pos = Int16(data=0)
        self.is_br_ready = Bool(data=False)
        self.match_started = Int16(data=0)

        self.manager = ISBManager(DEVICE_BUS, DEVICE_ADDR, DEVICE_NB_GPIOS, BLINK_PERIOD)
            
        for pin in LED_PINS:
            self.manager.initLed(pin)

        self.manager.setLedBlinking(LED_PINS[STATUS_LED_ID], True)

        for pin in BUTTONS_PINS:
            self.manager.initButton(pin)

        self.manager.initButton(TRIGGER_PIN)

        self.sub_idle = self.create_subscription(Int16, '/br/callbacks', self.display_idle, default_profile)
        
        self.pub_start = self.create_publisher(Int16, "/game/start", latch_profile)
        self.pub_color = self.create_publisher(Int16, '/game/color', latch_profile)
        self.pub_strat = self.create_publisher(Int16, '/game/strat', latch_profile)
        self.pub_init_pos = self.create_publisher(Int16, '/game/init_pos', latch_profile)
        self.pub_idle = self.create_publisher(Bool, '/br/idle', latch_profile)

        self.get_logger().info("ISB node initialized")

    def display_idle(self, msg):
        pin = LED_PINS[BR_IDLE_LED_ID]
        if msg.data == BR_Callback.OK_IDLE:
            self.manager.setLedState(pin, LedState.OFF)
        elif msg.data == BR_Callback.OK_READY:
            self.manager.setLedState(pin, LedState.ON)

    def update_strat(self):
        self.current_strat.data = (self.current_strat.data + 1) % self.strat_count
        self.pub_strat.publish(self.current_strat)
        self.get_logger().info(f"Update strat to {self.current_strat.data}")

    def update_color(self):
        self.current_color.data = 1 - self.current_color.data
        self.pub_color.publish(self.current_color)
        self.get_logger().info("Update color to " + COLOR[self.current_color.data])

    def update_init_pos(self):
        self.current_init_pos.data = (self.current_init_pos.data + 1) % self.init_pos_count
        self.pub_init_pos.publish(self.current_init_pos)
        self.get_logger().info(f"Update init zone to {self.current_init_pos.data}")

    def update_idle(self):
        self.is_br_ready.data = not self.is_br_ready.data
        self.pub_idle.publish(self.is_br_ready)
        if self.is_br_ready.data:
            self.get_logger().info("Set BR to Ready")
        else:
            self.get_logger().info("Set BR to Idle")

    def start_match(self):
        self.get_logger().info("Start match triggered")
        self.match_started.data = 1
        self.pub_start.publish(self.match_started)
        time.sleep(0.001)
        self.pub_start.publish(self.match_started)

        self.manager.setLedState(LED_PINS[TRIGGER_LED_ID], LedState.ON)

    def run(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0)
            self.manager.do_blink()

            if self.match_started.data == 0 and self.manager.getButtonState(TRIGGER_PIN) == 0:
                self.start_match()
                
            for pin in self.manager.check_buttons():
                if pin == TRIGGER_PIN:
                    if self.match_started.data == 0 and self.manager.getButtonState(TRIGGER_PIN) == 0:
                        self.start_match()
                elif pin == BUTTONS_PINS[STRAT_BUTTON_ID]:
                    self.update_strat()

                elif pin == BUTTONS_PINS[COLOR_BUTTON_ID]:
                    self.update_color()
                
                elif pin == BUTTONS_PINS[INIT_POS_BUTTON_ID]:
                    self.update_init_pos()

                elif pin == BUTTONS_PINS[BR_IDLE_BUTTON_ID]:
                    self.update_idle()
            
            time.sleep(0.01)
    
    def destroy_node(self):
        self.manager.setLedState(LED_PINS[STATUS_LED_ID], LedState.OFF)
        super().destroy_node()

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = ISBNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()