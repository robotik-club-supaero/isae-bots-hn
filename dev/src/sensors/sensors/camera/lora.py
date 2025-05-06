from typing import List

import RPi.GPIO as GPIO
import board
import busio
from digitalio import DigitalInOut 
import adafruit_rfm9x

class Lora:

    def __init__(self, logger, cs=board.CE1, reset=board.D25, interrupt=None, sck=board.SCK, frequency=868.0, **kwargs):
        self.logger = logger

        cs = DigitalInOut(cs)
        rst = DigitalInOut(reset)        
        spi = busio.SPI(sck, MOSI=board.MOSI, MISO=board.MISO)

        self._lora = adafruit_rfm9x.RFM9x(spi, cs, rst, frequency, **kwargs)
        self._lora.spreading_factor = 11 # Increase noise resistance
        self._lora.receive_timeout = 0.0

        self._supports_interrupts = interrupt is not None
        if interrupt is not None:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(interrupt, GPIO.IN, pull_up_down=GPIO.DOWN)
            GPIO.add_event_detect(interrupt, GPIO.RISING)
            GPIO.add_event_callback(interrupt, self._handle_interrupt)

    def setCallback(self, callback):
        if not self._supports_interrupts:
            self.logger.error("LORA: setCallback called but no pin was specified to handle interrupts. The callback will never be called.")
        self._on_recv = callback
    
    def _on_recv(self, message):
        pass

    def _handle_interrupt(self):
        if self._lora.rx_done():
            message = self.receive()
            if message is not None:
                self._on_recv(message)

    def receive(self, **kwargs):
        message = self._lora.receive(**kwargs)
        if message is not None:
            self.logger.debug(f"Received LoRa message: {message}")
            rssi = self._lora.last_rssi
            snr = self._lora.last_snr
            
            if rssi <= -120 or snr <= -13: # todo check if lib uses db
                self.logger.warn(f"Bad radio link metrics for received message: RSSI: {rssi}, SNR: {snr}")
        return message

    def send(self, message, **kwargs):
        return self._lora.send(message, **kwargs)

    def close(self):
        self._lora.sleep()
