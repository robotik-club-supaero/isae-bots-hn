# -*- coding: utf-8 -*-


'''
The ISB Manager only handles pins and buttons and doesn't know what they are for
Except for the trigger pin which is not exactly a button
'''

BLINK_PERIOD = 0.25 # s

NB_LEDS = 10
NB_BUTTONS = 5

LED_PINS = [15,14,13,12,11,10,9,8,7,6]
BUTTONS_PINS = [0,1,2,3,4]
TRIGGER_PIN = 5

# BUTTON_LEDS_IDS = [9,8,7,6,5]
# FREE_LEDS_IDS = [4,3,2,1]
# TRIGGER_LED_ID = 0

# COLOR_BUTTON_ID = 1
# BR_IDLE_BUTTON_ID = 2
# RESET_STEPPER_BUTTON_ID = 3