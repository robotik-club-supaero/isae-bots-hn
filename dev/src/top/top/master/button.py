import time
from enum import IntEnum
from abc import ABC, abstractmethod

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

        self._initState = GPIO.input(self.pin)

    def getButtonState(self):
        state = GPIO.input(self.pin)
        if state == self._initState:
            return ButtonState.OFF
        else:
            return ButtonState.ON

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

class BaseLed(ABC):
    """Base class for a Led with a blinking logic"""
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
        self._onChange()

    @abstractmethod
    def _onChange(self):
        pass

    def update(self):
        if self._blinking:
            if time.time() - self._lastBlink > self._blinkInterval:
                self._lastBlink = time.time()
                if self._state == LedState.ON:
                    self._setLedState(LedState.OFF)
                else:
                    self._setLedState(LedState.ON)

    @property
    def state(self):
        return self._state

class DummyLed(BaseLed):
    def _onChange(self):
        pass

class GpioLed(BaseLed):
    def __init__(self, pin):
        super().__init__()

        self.pin = pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)

    def _onChange(self):
        GPIO.output(self.pin, self.state == LedState.ON)
