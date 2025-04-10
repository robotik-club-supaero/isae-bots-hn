# -*- coding: utf-8 -*-

import serial
import time
import logging
logger = logging.getLogger(__name__)

from .nanoInterface import NanoCommand, NanoCallback, NanoEvent, TERMINAL_CHARACTER, ButtonColorMode, ButtonPressState

class ArduinoCommunicator:
    
    ledButtonState = -1  # not initialized
    
    def __init__(self, port='/dev/ttyNANO', baudrate=9600):
        self.ser = serial.Serial(port, baudrate)
        
        # read and write timeouts are None by default (but set for good measure)
        self.ser.timeout = None
        self.ser.write_timeout = None


    def send_command(self, nanoCommand, color=None, verbose=False):
        """
        Takes a NanoCommand object
        """
        
        if nanoCommand in [NanoCommand.CMD_COLOR_FIXED, NanoCommand.CMD_COLOR_BLINKING, NanoCommand.CMD_COLOR_FADING]:
            if color is None:
                logger.error("ERROR : give a color input")
                return NanoCallback.CLB_KO
            bytesToSend = bytearray([nanoCommand.value] + list(color) + [TERMINAL_CHARACTER])
            
        else:
            bytesToSend = bytearray([nanoCommand.value, TERMINAL_CHARACTER])
            
        self.ser.write(bytesToSend)
        # print("Sent raw command ", bytesToSend.decode())
        if verbose: logger.debug("Sent nano command " + nanoCommand.name)


    def receive_response(self, verbose=False):
        """
        Returns a NanoCallback object or None for an event
        """
        try:
            byte_received = self.ser.read(size=1)
        except serial.serialutil.SerialException:
            return NanoCallback.CLB_CONNEXION_FAILED
        
        res = int.from_bytes(byte_received, byteorder='little', signed=False)
        
        # check if data is an event and not a callback
        if res in [e.value for e in NanoEvent]:
            nanoEvent = NanoEvent(res)
            
            # change button state (#NOTE for now the only event)
            if nanoEvent == NanoEvent.EVENT_BUTTON_ON:
                self.ledButtonState = 1
            elif nanoEvent == NanoEvent.EVENT_BUTTON_OFF:
                self.ledButtonState = 0
            else:
                logger.error("ERROR : unknown event type")
                
            # print("Received button press event to new state ", self.ledButtonState)
                
            return self.receive_response()  #NOTE recursive so that we read until we find a callback or nothing
            #BUG possible de stack overflow si on a trop d'events
            #TODO ne pas faire en récursif
            
        # else it is a callback, we return the callback
        try:
            nanoCallback = NanoCallback(res)
            
            if verbose: logger.debug("Received nano callback " + nanoCallback.name)
            return nanoCallback
        except:
            logger.warn("Received unknown nano callback: " + str(res))
            return NanoCallback.CLB_CONNEXION_FAILED
    
    
    def establish_communication(self):
        
        logger.info("Waiting for communication to be established ...")
            
        # set read timeout to a value for non blocking reads
        self.ser.timeout = 0.1

        while True: # wait for communication to be established
            
            self.send_command(NanoCommand.CMD_INIT)
            
            nanoCallback = self.receive_response()  # times out after 1 second
            
            if nanoCallback is None: continue  # an event is not handled at this point
            
            if nanoCallback == NanoCallback.CLB_INIT_OK:
                logger.info("Connection established")
                return 0
            elif nanoCallback == NanoCallback.CLB_CONNEXION_FAILED:
                return 1
            
            time.sleep(0.01)
            
            
    def read_button_state(self):
        """
        Returns the current button state or None if cannot read it
        """
        
        self.send_command(NanoCommand.CMD_READ_BUTTON)
        nanoCallback = self.receive_response()
        
        # print("callback : ", nanoCallback)
        
        if nanoCallback is None:
            return None
        
        #TODO fix ON/OFF discrepancy
        if nanoCallback.value == NanoCallback.CLB_BUTTON_ON:
            return ButtonPressState.BUTTON_PRESS_OFF
        elif nanoCallback.value == NanoCallback.CLB_BUTTON_OFF:
            return ButtonPressState.BUTTON_PRESS_ON
            
        # else:
        #     print("ERROR : wrong button event callback")
        #     return None
                
        
        
    def inputSendCommand(self):
        """
        Test
        """
        intCommand = int( input('Enter int command : ') )
        nanoCommand = NanoCommand(intCommand)
        
        if nanoCommand in [NanoCommand.CMD_COLOR_FIXED, NanoCommand.CMD_COLOR_BLINKING]:
            R = int( input('Red : ') )
            G = int( input('Green : ') )
            B = int( input('Blue : ') )
            
            self.send_command(nanoCommand, color=[R, G, B])
        
        else:
            self.send_command(nanoCommand)
            
        nanoCallback = self.receive_response()
        
        
    def changeButtonColor(self, buttonColorMode, color=None):
        
        if buttonColorMode in [ButtonColorMode.BUTTON_COLOR_STATIC, ButtonColorMode.BUTTON_COLOR_BLINKING, ButtonColorMode.BUTTON_COLOR_FADING]:
            
            if color is None:
                logger.error("ERROR : color is None for colored ButtonState")
            
            if buttonColorMode == ButtonColorMode.BUTTON_COLOR_STATIC:
                self.send_command(NanoCommand.CMD_COLOR_FIXED, color=color)
            elif buttonColorMode == ButtonColorMode.BUTTON_COLOR_BLINKING:
                self.send_command(NanoCommand.CMD_COLOR_BLINKING, color=color)
            else:
                self.send_command(NanoCommand.CMD_COLOR_FADING, color=color)
                
            
        elif buttonColorMode == ButtonColorMode.BUTTON_COLOR_NYAN:
            self.send_command(NanoCommand.CMD_NYAN)
            
        else:
            logger.error("ERROR : cannot process this button state")
