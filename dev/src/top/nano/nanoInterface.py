# -*- coding: utf-8 -*-

from enum import IntEnum

TERMINAL_CHARACTER = 120  # ascii code for 'x'

class NanoCommand(IntEnum):
    CMD_INIT = 0
    CMD_COLOR_FIXED = 1
    CMD_COLOR_BLINKING = 2

    CMD_RAINBOW = 3
    CMD_NYAN = 4

    CMD_READ_BUTTON = 5
    
    
class NanoCallback(IntEnum):
    CLB_OK = 0
    CLB_KO = 1
    CLB_INIT_OK = 2

    CLB_BUTTON_ON = 50
    CLB_BUTTON_OFF = 51

    CLB_NO_BYTES_READ = 98
    CLB_BUFFER_OVERFLOW = 99

    CLB_UNKNOWN_COMMAND = 100
    
    
class NanoEvent(IntEnum):
    EVENT_BUTTON_ON = 202
    EVENT_BUTTON_OFF = 203

