import time
from enum import IntEnum

import RPi.GPIO as GPIO

DUMMY_BUTTON_TRIGGER_ON_TIMEOUT = 10
DUMMY_BUTTON_TRIGGER_OFF_TIMEOUT = 15

class ButtonState(IntEnum):
    OFF = 0
    ON = 1

LedState = ButtonState

class GpioButton:
    def __init__(self, pin):
        self.pin = pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN)

    def getButtonState(self):
        state = GPIO.input(self.pin)
        if state == 1:
            return ButtonState.ON
        else:
            return ButtonState.OFF


class DummyButton:
    def __init__(self):
        self._init = time.time()
        self._state = ButtonState.OFF

    def getButtonState(self):
        if self._state == ButtonState.OFF and time.time() - self._init > DUMMY_BUTTON_TRIGGER_ON_TIMEOUT:
            self._state = ButtonState.ON
            self._init = time.time()
        elif self._state == ButtonState.ON and time.time() - self._init > DUMMY_BUTTON_TRIGGER_OFF_TIMEOUT:
            self._state = ButtonState.OFF
            self._init = time.time()
        
        return self._state

class DummyLed:
    def __init__(self):
        self._setLedState(LedState.OFF)
        self._lastBlink = 0
        self._blinking = False
        self._blinkInterval = 0

    def setLedState(self, state):
        self._setLedState(state)
        self._blinking = False

    def setLedBlinking(self, interval):
        self._blinking = interval > 0
        self._setLedState(LedState.OFF)
        self._blinkInterval = interval

    def _setLedState(self, state):
        self._state = state

    def update(self):
        if self._blinking:
            if time.time() - self._lastBlink > self._blinkInterval:
                self._lastBlink = time.time()
                if self._state == LedState.ON:
                    self._setLedState(LedState.OFF)
                else:
                    self._setLedState(LedState.ON)