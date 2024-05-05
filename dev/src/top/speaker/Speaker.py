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
import threading

from .speaker_const import soundDict, SOUNDS_PATH, BLANK_SOUND_FILE


class Speaker():
    
    media_player = None
    
    volume = 100  # by default
    isMute = False  #NOTE by default

    
    def __init__(self) -> None:
        
        self.media_player = vlc.MediaPlayer()
        
        # thread to play a counstant blank sound to prevent the speaker to do any poppins sounds
        self.constantBlankSoundStopEvent = threading.Event()
        self.constantBlankSoundThread = threading.Thread(target=self.constantBlankSound, args=(self.constantBlankSoundStopEvent,))
        
        self.constantBlankSoundThread.start()
        
        self.media_player.audio_set_volume(100) # default volume
        
        
    def constantBlankSound(self, stop_event):
        
        player = vlc.Instance()
        
        mediaList = player.media_list_new()
        mediaList.add_media(SOUNDS_PATH + BLANK_SOUND_FILE)
        listPlayer = player.media_list_player_new()
        listPlayer.set_media_list(mediaList)
        listPlayer.set_playback_mode(vlc.PlaybackMode(1))
        
        listPlayer.play()
                
        # Wait for stop event to end sound
        while not stop_event.is_set():
            time.sleep(0.1)

        listPlayer.stop()        
        
    
    
    def __startAudio(self, source):
        
        # media resource locator
        mrl = source
        
        # setting mrl to the media player
        self.media_player.set_mrl(mrl)
        
        # set volume (in case it has been changed)
        # self.media_player.audio_set_volume(self.volume)
        
        # start playing video
        self.media_player.play()
        
        
        
    def playSound(self, sound):
        
        try:
            soundFile = soundDict[sound]
        except KeyError:
            print("ERROR : key is not in SoundDict")
            return
            
        soundPath = SOUNDS_PATH + soundFile
        
        # check if the sound file exists
        if not os.path.isfile(soundPath):
            print(f"ERROR : sound file {soundPath} does not exist")
            return
                
        #BUG possible si deux sons sont joués quasi en même temps
        # On peut temporiser plus haut dans le topServer
        # -> mettre un sleep de 0.01 entre les sons pour éviter les erreurs vlc (sécurité)
        # uniquement dans le thread des sons
        
        print(f"Playing sound {soundFile}")
        self.__startAudio(soundPath)
        
        time.sleep(0.01)  # safety
        
        
    def setVolume(self, volume):
        print(f"Set sound volume to {volume}")
        self.volume = volume
        
                
        
    def setMute(self, isMute):
        
        # if no change
        if isMute == self.isMute:
            return
                
        
        self.isMute = isMute
        
        #TOTEST choisir le type de mute qui marche le mieux sur la pi
        # sur le PC les deux font la même chose
        if isMute:
            # self.media_player.audio_set_volume(0)
            self.media_player.audio_set_mute(True)
        else:
            # self.media_player.audio_set_volume(self.volume)
            self.media_player.audio_set_mute(False)
            
        #TODO mute or unmute
        
        # self.media_player.audio_set_mute()
        