
from enum import IntEnum
import time

from .PCA95XX import PCA95XX

class LedState(IntEnum):
    OFF = 0
    ON = 1

class PinMode(IntEnum):
    INPUT = 1
    OUTPUT = 0

class ISBManager:
    def __init__(self, device_bus, device_addr, device_nb_gpios, blink_period):
        self.blink_period = blink_period # s

        self._pca95xx = PCA95XX(device_bus, device_addr, device_nb_gpios)
        self._blink_timer = time.perf_counter()
        self._ledState = {}
        self._buttonState = {}

    def initLed(self, pin):
        self._pca95xx.config(pin=pin, mode=PinMode.OUTPUT.value)
        self._ledState[pin] = {"state": LedState.OFF, "blinking": False}

    def initButton(self, pin):
        self._pca95xx.config(pin=pin, mode=PinMode.INPUT.value)
        self._buttonState[pin] = self.getButtonState(pin)

    def setLedState(self, pin, state, stop_blinking=True):
        if stop_blinking:
            self._ledState[pin]["blinking"] = False
        
        self._pca95xx.output(pin=pin, value=state.value)
        self._ledState[pin]["state"] = state

    def setLedBlinking(self, pin, blink):
        self._ledState[pin]["blinking"] = blink

    def getButtonState(self, pin):
        return 0 if self._pca95xx.input(pin=pin) == 0 else 1

    def do_blink(self):
        now = time.perf_counter()
        if now - self._blink_timer > self.blink_period:
            self._blink_timer = now

            for pin in self._ledState.keys():
                state = self._ledState[pin]
                if state["blinking"]:
                    self.setLedState(pin, LedState(1 - state["state"].value), stop_blinking=False)

    def check_buttons(self):
        for pin in self._buttonState.keys():
            state = self._buttonState[pin]
            new_state = self.getButtonState(pin)
            if new_state != state:
                self._buttonState[pin] = new_state
                yield pin