#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import time
import vlc
 
# method to play video
def audio(source):
     
    # creating a vlc instance
    vlc_instance = vlc.Instance()
    
    # creating a media player
    player = vlc_instance.media_player_new()
     
    # creating a media
    media = vlc_instance.media_new(source)
     
    # setting media to the player
    player.set_media(media)
     
    # play the video
    player.play()
     
    # wait time
    time.sleep(5)
     
    # getting the duration of the video
    duration = player.get_length()
     
    # printing the duration of the video
    print("Duration : " + str(duration))
     

def start_audio(media_player, source):

    # media resource locator
    mrl = source
    
    # setting mrl to the media player
    media_player.set_mrl(mrl)
    
    # start playing video
    media_player.play()
    


def main():

    #dummy_media = vlc.MediaPlayer()
    #dummy_media.audio_set_volume(0)
    #start_audio(dummy_media, "Darude - Sandstorm.mp3")


    # time.sleep(10)

    media_player = vlc.MediaPlayer()

    print(media_player.audio_get_volume())
    media_player.audio_set_volume(100)
    print(f"Media : {media_player.get_media()}")
    if media_player.get_media() is None:
        print("No media has been loaded yet")
    
    # NOTE : this way the plays are buffered
    start_audio(media_player, "windowsXPStartup.mp3") #TODO catch cette erreur
    print(f"Media : {media_player.get_media()}")
    time.sleep(4000)

    print(media_player.audio_get_volume())
    #media_player.audio_set_volume(120)
    time.sleep(0.5)

    start_audio(media_player, "Oh merde oh cest con ça.mp3")
    time.sleep(4)

    print(media_player.audio_get_volume())


    # if we want multiple plays at the same time we can create many instances of MediaPlayer


if __name__ == "__main__":
    main()
