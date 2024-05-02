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
import signal

from .isb_lib import initPin, readPin, writePin
from .isb_const import *


def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    print("Node forced to terminate ...")
    sys.exit(1)



#######################################################################
# ISB NODE
#######################################################################

class ISBManager:
    """
    ISB node.
    """

    def __init__(self):	

        # --- Variables
        self.blinkTimer = time.perf_counter()
        
        self.buttonStates = [-1]*NB_BUTTONS
        self.isButtonTriggered = [False]*NB_BUTTONS
        self.ledStates = [0]*NB_LEDS  # 0 : OFF | 1 : ON
        self.isLEDBlinking = [False]*NB_LEDS

        self.initPins()

        self.setAllLeds(0)

        print("ISB Manager initialized")



    # --- Pixel handling functions

    def initPins(self):

        for id_pin in LED_PINS:
            initPin(id_pin, "output")

        for id_pin in BUTTONS_PINS:
            initPin(id_pin, "input")

        initPin(TRIGGER_PIN, "input")



    def writeLed(self, led_id, ledState):
        '''
        state of 0 : OFF | 1 : ON | 2 : activate blinking
        '''
        
        # if activate blinking just set it and return
        if ledState == 2:
            self.isLEDBlinking[led_id] = True
            return
            
        self.isLEDBlinking[led_id] = False
        writePin(LED_PINS[led_id], ledState)


    
    def setAllLeds(self, state):
        for k in range(NB_LEDS):
            self.writeLed(k, state)


    def readButtonStates(self):
        
        for k in range(NB_BUTTONS):
            self.buttonStates[k] = readPin(BUTTONS_PINS[k])
        
        return self.buttonStates

        # for k in range(NB_BUTTONS):
        #     res = readPin(BUTTONS_PINS[k])

        #     # update button triggered list
        #     self.isButtonTriggered[k] = self.buttonStates[k] != res
        #     self.buttonStates[k] = res

        #     # apply state to corresponding led
        #     self.writeLed(BUTTON_LEDS_IDS[k], self.buttonStates[k])


    def readTriggerState(self):

        res = readPin(TRIGGER_PIN)

        # # only change trigger state if previous state is not -1
        # if res == 1:
        #     self.triggerState = 1
        #     self.writeLed(TRIGGER_LED_ID, self.triggerState)  # trigger is ready to be pulled

        # if self.triggerState == 1 and res == 0:
        #     self.triggerState = 0  # triggers match starts
            
        return res


    # # --- Callback functions

    # def callBackBR(self, msg):

    #     if msg.data == 5:  # OK_READY
    #         self.writeLed(BR_IDLE_LED_ID, 1)

    #     elif msg.data == 6:  # OK_IDLE
    #         self.writeLed(BR_IDLE_LED_ID, 0)


    def run(self):
        
        while True:
        
            # blink LEDs in blink state
            if time.perf_counter() - self.blinkTimer > BLINK_PERIOD:
                
                for k in range(NB_LEDS):
                    if self.isLEDBlinking[k]:
                        self.ledStates[k] = 1 - self.ledStates[k]
                        self.writeLed(k, self.ledStates[k])
                
                self.blinkTimer = time.perf_counter()
                
            time.sleep(0.01)