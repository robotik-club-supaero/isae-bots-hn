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

from nano.ArduinoCommunicator import ArduinoCommunicator
from speaker.Speaker import Speaker

import socket

TOPSERVER_PORT = 5678

class TopServer():
    
    nanoCom = None  # nano board
    
    server = None  # TCP server
    
    def __init__(self) -> None:
        
        self.nanoCom = ArduinoCommunicator(port='/dev/ttyUSB0', baudrate=9600) #TODO give a linked name to the arduino Nano
    
        self.speaker = Speaker()
    
        self.watchButtonThread = threading.Thread(target=self.watchButton, args=(1,))
        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # listen on port
        self.server.bind( ("0.0.0.0", TOPSERVER_PORT) )
        maxclients = 10
        self.server.listen(maxclients)
    
    
    def watchButton(self, arg):
        
        res = self.nanoCom.receive_response()
        
        while True:
            res = self.nanoCom.receive_response()
            print("res from thread : ", res)
            
            self.speaker.play_sound("startup1.mp3")
            
    
    
    def run(self):
        
        self.watchButtonThread.start()
        
        # while True:
        #     self.nanoCom.inputSendCommand()
        
        
        
    def closeServer(self):
        
        self.server.close()
        
        #TODO gros signal d'erreur (lumière rouge ou jsp) si le topServer n'est pas actif (pas d'ISB)
        

def main():
    
    topserver = TopServer()
    
    topserver.run()


if __name__ == '__main__':
    main()