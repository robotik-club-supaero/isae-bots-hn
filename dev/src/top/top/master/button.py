import time
from enum import IntEnum

DUMMY_BUTTON_TRIGGER_TIMEOUT = 2

class ButtonState(IntEnum):
    OFF = 0
    ON = 1

LedState = ButtonState

class DummyButton:
    def __init__(self):
        self._init = time.time()

    def getButtonState(self):
        if time.time() - self._init < DUMMY_BUTTON_TRIGGER_TIMEOUT or time.time() - self._init > 5 * DUMMY_BUTTON_TRIGGER_TIMEOUT:
            return ButtonState.OFF
        else:
            return ButtonState.ON

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