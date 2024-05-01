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

import threading
import time
import socket

#### Nano ####
from nano.ArduinoCommunicator import ArduinoCommunicator

#### Speaker ####
from speaker.Speaker import Speaker

#### ISB ####
# from isb.isb_lib import initPin, readPin, writePin
# from isb.isb_const import *

from top_const import TOPSERVER_PORT, MIN_TIME_FOR_BUTTON_CHANGE


class TopServer():
    
    nanoCom = None  # nano board
    
    server = None  # TCP server
    
    def __init__(self) -> None:
        
        self.nanoCom = ArduinoCommunicator(port='/dev/ttyUSB0', baudrate=9600) #TODO give a linked name to the arduino Nano
    
        self.speaker = Speaker()
    
        self.watchButtonThread = threading.Thread(target=self.watchButton, args=(1,))
        self.buttonState, self.previousButtonState = None, None
        self.lastButtonChangeTime = time.perf_counter()
                        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # listen on port
        self.server.bind( ("0.0.0.0", TOPSERVER_PORT) )
        maxclients = 10
        self.server.listen(maxclients)
        
        #TODO vérifier les branchements des éléments top et des autres périphériques de la rpi
        # signaler s'il y a un pb
    
    
    
    def initButton(self):
        
        self.nanoCom.establish_communication()
        
        self.buttonState = self.nanoCom.read_button_state()
        self.previousButtonState = self.buttonState

    
    def watchButton(self, arg):
                
        while True:
            
            self.buttonState = self.nanoCom.read_button_state()
            
            if self.previousButtonState != self.buttonState:
                
                # filter on button changes to avoid fast toggling
                if time.perf_counter() - self.lastButtonChangeTime < MIN_TIME_FOR_BUTTON_CHANGE:
                    continue
                
                self.lastButtonChangeTime = time.perf_counter()
                
                if self.buttonState == 0 :
                    print("Button OFF")    
 
                elif self.buttonState == 1 :
                    print("Button ON")
                    
                else:
                    print(f"ERROR : unknown button callback {self.buttonState}")
                    continue
                
            self.previousButtonState = self.buttonState


            time.sleep(0.01)
        
        while True:
            res = self.nanoCom.receive_response()
            print("res from thread : ", res)
            
            # self.speaker.play_sound("startup1.mp3")
            time.sleep(0.01)
            
            
    def receiveCommand(self):
        
        res = self.server.recv(1024)
        
        #TODO split command into different cases
            
            
    def soundCommand(self, command):
        '''Command received from the socket'''
        
        # play sound command
        
        # set volume command
        
        # set mute command
        
        return
    
    
    def run(self):
        
        #TODO gros signal d'erreur si le topServer n'est pas actif (on n'aurait pas d'ISB)
        
        print('Running TopServer')
        
        self.watchButtonThread.start()
        
        while True:
            time.sleep (0.01)
        
        # while True:
        #     self.nanoCom.inputSendCommand()
        
        
        
    def closeServer(self):
        
        self.server.close()
        
        

def main():
    
    topserver = TopServer()
    
    topserver.initButton()
    
    topserver.run()

    
    
    '''Test'''
    time.sleep(0.5)
    
    # topserver.speaker.setVolume(120)
        
    topserver.speaker.playSound('windowsStartup')
    
    time.sleep(1)
    
    topserver.speaker.setMute(True)
    
    time.sleep(1)
    
    topserver.speaker.setMute(False)
    
    time.sleep(3)
    
    topserver.speaker.playSound('startup')
    
    time.sleep(50)
    


if __name__ == '__main__':
    main()