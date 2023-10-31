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


import time
import vlc


SOUNDS_PATH = 'speaker/sounds/'

class Speaker():
    
    media_player = None
    isMute = None
    
    def __init__(self) -> None:
        
        self.media_player = vlc.MediaPlayer()
        
        self.media_player.audio_set_volume(100)
        
        self.isMute = False  #NOTE by default
        
    
    
    def __startAudio(self, source):
        
        # media resource locator
        mrl = source
        
        # setting mrl to the media player
        self.media_player.set_mrl(mrl)
        
        # start playing video
        self.media_player.play()
        
        
        
        
    def playSound(self, sound):
        
        #TODO check if the sound file exists
        
        
        print("Start sound")
        self.__startAudio(SOUNDS_PATH + sound)
        time.sleep(5)
        print("End sound")
        
        
    def setMute(self, isMute):
        
        # if no change
        if isMute == self.isMute:
            return
                
        
        self.isMute = isMute
            
        #TODO mute or unmute
        