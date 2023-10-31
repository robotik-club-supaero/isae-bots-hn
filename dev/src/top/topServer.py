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
# from nano.ArduinoCommunicator import ArduinoCommunicator

#### Speaker ####
from speaker.Speaker import Speaker

#### ISB ####
# from isb.isb_lib import initPin, readPin, writePin
# from isb.isb_const import *

from top_const import *


class TopServer():
    
    nanoCom = None  # nano board
    
    server = None  # TCP server
    
    def __init__(self) -> None:
        
        # self.nanoCom = ArduinoCommunicator(port='/dev/ttyUSB0', baudrate=9600) #TODO give a linked name to the arduino Nano
    
        self.speaker = Speaker()
    
        self.watchButtonThread = threading.Thread(target=self.watchButton, args=(1,))
        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # listen on port
        self.server.bind( ("0.0.0.0", TOPSERVER_PORT) )
        maxclients = 10
        self.server.listen(maxclients)
        
        #TODO vérifier les branchements des éléments top et des autres périphériques de la rpi
        # signaler s'il y a un pb
    
    
    def watchButton(self, arg):
        
        res = self.nanoCom.receive_response()
        
        while True:
            res = self.nanoCom.receive_response()
            print("res from thread : ", res)
            
            self.speaker.play_sound("startup1.mp3")
            
    
    
    def run(self):
        
        #TODO gros signal d'erreur si le topServer n'est pas actif (on n'aurait pas d'ISB)
        
        self.watchButtonThread.start()
        
        # while True:
        #     self.nanoCom.inputSendCommand()
        
        
        
    def closeServer(self):
        
        self.server.close()
        
        

def main():
    
    topserver = TopServer()
    
    # topserver.speaker.setVolume(120)
        
    topserver.speaker.playSound('bip.mp3')
    
    # time.sleep(5)
    
    topserver.speaker.playSound('startup1.mp3')
    
    time.sleep(50)
    
    # topserver.run()


if __name__ == '__main__':
    main()