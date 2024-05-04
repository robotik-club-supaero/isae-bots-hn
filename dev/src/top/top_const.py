# -*- coding: utf-8 -*-


from enum import IntEnum

TOPSERVER_PORT = 5678

MIN_TIME_FOR_BUTTON_CHANGE = 0.5


class ButtonColorMode(IntEnum):
    BUTTON_COLOR_UNKNOWN = -1
    BUTTON_COLOR_STATIC = 0
    BUTTON_COLOR_BLINKING = 1
    BUTTON_COLOR_FADING = 3
    BUTTON_COLOR_NYAN = 4
    
class ButtonPressState(IntEnum):
    BUTTON_PRESS_UNKNOWN = -1
    BUTTON_PRESS_OFF = 0
    BUTTON_PRESS_ON = 1
    
    
class TopServerRequest(IntEnum):
    REQUEST_READ_BUTTONS = 1
    REQUEST_READ_TRIGGER = 2
    REQUEST_WRITE_LED = 3
    REQUEST_WRITE_LEDBUTTON = 4
    
class TopServerCallback(IntEnum):
    CALLBACK_OK = 1
    CALLBACK_WRONG_ARGUMENTS = 2
    CALLBACK_TIMEOUT = 3
    
class RosLaunchState (IntEnum):
    ROSLAUNCH_STOPPED = 0
    ROSLAUNCH_STARTED = 1
    ROSLAUNCH_READY = 2
    ROSLAUNCH_STOPPING = 3
