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

import os
import time
import vlc


SOUNDS_PATH = os.path.join(os.path.dirname(__file__),'sounds/')

class Speaker():
    
    media_player = None
    
    volume = 100  # by default
    isMute = False  #NOTE by default

    
    def __init__(self) -> None:
        
        self.media_player = vlc.MediaPlayer()
        
        self.media_player.audio_set_volume(100)
        
        
    
    
    def __startAudio(self, source):
        
        # media resource locator
        mrl = source
        
        # setting mrl to the media player
        self.media_player.set_mrl(mrl)
        
        # set volume (in case it has been changed)
        self.media_player.audio_set_volume(self.volume)
        
        # start playing video
        self.media_player.play()
        
        
        
        
    def playSound(self, sound):
        
        # check if the sound file exists
        if not os.path.isfile(SOUNDS_PATH + sound):
            print(f"ERROR : sound file {sound} does not exist")
            return
                
        #BUG possible si deux sons sont joués quasi en même temps
        # On peut temporiser plus haut dans le topServer
        # -> mettre un sleep de 0.01 entre les sons pour éviter les erreurs vlc (sécurité)
        # uniquement dans le thread des sons
        
        print(f"Playing sound {sound}")
        self.__startAudio(SOUNDS_PATH + sound)
        
        time.sleep(0.01)  # safety
        
        
    def setVolume(self, volume):
        print(f"Sett sound volume to {volume}")
        self.volume = volume
        
                
        
    def setMute(self, isMute):
        
        # if no change
        if isMute == self.isMute:
            return
                
        
        self.isMute = isMute
            
        #TODO mute or unmute
        