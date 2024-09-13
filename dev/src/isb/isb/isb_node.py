#!/usr/bin/env python3
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

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import signal
import socket
import configparser
from ast import literal_eval

from std_msgs.msg      import Int16, Empty, Int16MultiArray
from geometry_msgs.msg import Quaternion

from top.top_const import connectToServer, sendRequest, receiveCallback, TopServerRequest, TopServerCallback

# CONFIG 
READER = configparser.ConfigParser()
READER.read(os.path.join(os.path.dirname(__file__),'../robot_config.cfg'))

#######################################################################
# ISB NODE
#######################################################################

REFRESH_PERIOD = 0.1 # s


NB_LEDS = 10
NB_BUTTONS = 5


LED_PINS = [15,14,13,12,11,10,9,8,7,6]
BUTTONS_PINS = [0,1,2,3,4]
TRIGGER_PIN = 5

BUTTON_LEDS_IDS = [9,8,7,6,5]
TRIGGER_LED_ID = 0
BR_IDLE_LED_ID = 1

STRAT_BUTTON_ID = 0
COLOR_BUTTON_ID = 1
BR_IDLE_BUTTON_ID = 2
RESET_STEPPER_BUTTON_ID = 3
INIT_ZONE_BUTTON_ID = 4

class ISBNode(Node):
    """
    ISB node.
    """

    def __init__(self):	
        super().__init__("ISB")
        self.log_info("Initializing ISB Node ...")

        # --- Variables
        self.currentTime = 0.0

        self.match = False
        self.color = 0
        self.init_zone = int(READER.get("STRAT", "init_zone"))
        self.strat = int(READER.get("STRAT", "strat_default"))
        self.strategies = list(literal_eval(READER.get('STRAT', 'strat_list')))
        self.nbStrats = len(self.strategies)

        self.buttonStates = [-1]*NB_BUTTONS
        self.isButtonTriggered = [False]*NB_BUTTONS
        self.triggerState = -1

        self.brIdle = -1

        self.isBlinking = [True, True, False, False, False, False, True, True]

        # --- Publishers & subscribers
        latch_qos = QoSProfile(
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Transient Local durability
        )
        self.pubStart = self.create_publisher(Int16, "/game/start", latch_qos)
        self.pubColor = self.create_publisher(Int16, "/game/color", latch_qos)
        self.pubStrat = self.create_publisher(Int16, "/game/strat", latch_qos)
        self.pubInitPos = self.create_publisher(Int16, "/game/init_pos", latch_qos)

        self.pubBRIdle = self.create_publisher(Int16, "/br/idle", latch_qos)
        self.pubResetStepper = self.create_publisher(Int16, "/strat/elevator", latch_qos)

        self.subIsbMatch = self.create_subscription(Int16, "/okPosition", self.callBackBR, QoSProfile())
  
        self.topServerSocket = connectToServer()
        
        self.create_timer(0.01, self.refresh)

        # init all LEDs to blinking
        for k in range(NB_LEDS):
            self.writeLed(k, 2)

        self.log_info("ISB node is ready")


    def writeLed(self, led_id, state):
    
        try:
            data = [TopServerRequest.REQUEST_WRITE_LED, led_id, state]
            sendRequest(self.topServerSocket, data)
            callback = receiveCallback(self.topServerSocket) 
            #NOTE the callback has to be received to not interfere with other callbacks (even if not used)
                                    
        except BrokenPipeError: #NOTE happens if the server shuts down by Ctrl-C
            print("Connexion to server stopped")


    def readButtons(self):
        
        try:            
            data = [TopServerRequest.REQUEST_READ_BUTTONS]
            sendRequest(self.topServerSocket, data)
            callback_data = receiveCallback(self.topServerSocket)
        
        except BrokenPipeError: #NOTE happens if the server shuts down by Ctrl-C
            self.log_warn("Connexion to server stopped")
                        
        if callback_data is None:
            print("Couldn't read buttons")
            return
        
        res = callback_data[1:]
            
        for k in range(NB_BUTTONS):

            # update button triggered list
            self.isButtonTriggered[k] = self.buttonStates[k] != res[k]

            # # apply state to corresponding led if it was triggered
            if self.isButtonTriggered[k]:
                self.buttonStates[k] = res[k]
                self.writeLed(BUTTON_LEDS_IDS[k], self.buttonStates[k])


    def readTrigger(self):
        
        try:            
            data = [TopServerRequest.REQUEST_READ_TRIGGER]
            sendRequest(self.topServerSocket, data)
            callback_data = receiveCallback(self.topServerSocket)
                    
        except BrokenPipeError: #NOTE happens if the server shuts down by Ctrl-C
            self.log_warn("Connection to server stopped")
                            
        if callback_data is None:
            print("Couldn't read trigger")
            return
        
        res = callback_data[1]

        # only change trigger state if previous state is not -1
        if res == 1:
            self.triggerState = 1
            self.writeLed(TRIGGER_LED_ID, self.triggerState)  # trigger is ready to be pulled

        if self.triggerState == 1 and res == 0:
            self.triggerState = 0  # triggers match starts




    def callBackBR(self, msg):

        if msg.data == 5:  # OK_READY
            self.writeLed(BR_IDLE_LED_ID, 1)

        elif msg.data == 6:  # OK_IDLE
            self.writeLed(BR_IDLE_LED_ID, 0)

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

    def refresh(self):

        if time.perf_counter() - self.currentTime > REFRESH_PERIOD:

            self.readButtons()
            self.readTrigger()

            # Send match message if the trigger is released
            if  not self.match and self.triggerState == 0:
                self.log_info("Match started")
                self.match = True
                self.pubStart.publish(data=1)
                self.pubStart.publish(data=1)  # another one to be sure
                
                data = [TopServerRequest.REQUEST_PLAY_SOUND, "cestParti"]
                sendRequest(self.topServerSocket, data)
                callback = receiveCallback(self.topServerSocket)
                #NOTE the callback has to be received to not interfere with other callbacks (even if not used)

            # Send color message
            if self.isButtonTriggered[COLOR_BUTTON_ID]:

                if self.color == 0 and self.buttonStates[COLOR_BUTTON_ID] == 1:
                    self.log_info("Set color to AWAY")
                    self.color = 1
                    self.pubColor.publish(data=1)


                elif self.color == 1 and self.buttonStates[COLOR_BUTTON_ID] == 0:
                    self.log_info("Set color to HOME")
                    self.color = 0
                    self.pubColor.publish(data=0)
                    
            
            # Send strat message
            if self.isButtonTriggered[STRAT_BUTTON_ID]:
                
                self.strat += 1
                
                if self.strat >= self.nbStrats:
                    self.strat = 0
                    
                self.log_info(f"Set strat to {self.strat}")
                
                self.pubStrat.publish(data=self.strat)

            # Send init zone message
            if self.isButtonTriggered[INIT_ZONE_BUTTON_ID]:

                self.init_zone += 1
                if self.init_zone > 2:
                    self.init_zone = 0
                
                self.log_info(f"Set init zone to {self.init_zone}")  

                self.pubInitPos.publish(data=self.init_zone)

            # Send BR idle message
            if self.isButtonTriggered[BR_IDLE_BUTTON_ID]:

                if self.brIdle == 1 and self.buttonStates[BR_IDLE_BUTTON_ID] == 0:
                    self.log_info("Set BR to Idle")
                    self.brIdle = 0
                    self.pubBRIdle.publish(data=0)

                elif self.brIdle == 0 and self.buttonStates[BR_IDLE_BUTTON_ID] == 1:
                    self.log_info("Set BR to Ready")
                    self.brIdle = 1
                    self.pubBRIdle.publish(data=1)
                else:
                    self.brIdle = self.buttonStates[BR_IDLE_BUTTON_ID]

            # Reset stepper message
            if self.isButtonTriggered[RESET_STEPPER_BUTTON_ID]:
                self.log_info("Reset stepper position")
                self.pubResetStepper.publish(data=-1)

            self.currentTime = time.perf_counter()
        

#######################################################################
# LAUNCH
#######################################################################

def main():
    rclpy.init(args=sys.argv)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    node = ISBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_warn("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()