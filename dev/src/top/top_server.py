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


class TopServer():
    
    nano = None  # nano board
    
    def __init__(self) -> None:
        
        self.nano = ArduinoCommunicator(port='/dev/ttyUSB0', baudrate=9600) #TODO give a linked name to the arduino Nano
    
        self.speaker = Speaker()
    
        self.thread1 = threading.Thread(target=self.watchButton, args=(1,))
    
    
    def watchButton(self, arg):
        
        res = self.nano.receive_response()
        
        while True:
            res = self.nano.receive_response()
            print("res from thread : ", res)
            
            self.speaker.play_sound("startup1.mp3")

    
    
    def run(self):
        
        self.thread1.start()
        
        # while True:
        #     self.nano.inputSendCommand()
        
        

def main():
    
    topserver = TopServer()
    
    topserver.run()

if __name__ == '__main__':
    main()