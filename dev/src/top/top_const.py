# -*- coding: utf-8 -*-


from enum import IntEnum

TOPSERVER_PORT = 5678

MIN_TIME_FOR_BUTTON_CHANGE = 0.5


class ButtonColorMode(IntEnum):
    BUTTON_COLOR_UNKNOWN = -1
    BUTTON_COLOR_STATIC = 0
    BUTTON_COLOR_BLINKING = 1
    BUTTON_COLOR_NYAN = 4
    
class ButtonPressState(IntEnum):
    BUTTON_PRESS_UNKNOWN = -1
    BUTTON_PRESS_OFF = 0
    BUTTON_PRESS_ON = 1