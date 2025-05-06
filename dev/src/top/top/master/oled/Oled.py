import os

import board
import busio
import digitalio
import adafruit_ssd1306

from PIL import Image

IMAGES_PATH = f"{os.path.dirname(os.path.realpath(__file__))}/images/"

class OledScreen:

    WIDTH = 128
    HEIGHT = 64
    TOP_BORDER = 15 # Top of screen is degraded

    def __init__(self, addr=0x3d, oled_reset=None):
        if oled_reset is not None:
            oled_reset = digitalio.DigitalInOut(oled_reset)

        i2c = busio.I2C(board.SCL, board.SDA)
        self._oled = adafruit_ssd1306.SSD1306_I2C(OledScreen.WIDTH, OledScreen.HEIGHT, i2c, addr=addr, reset=oled_reset)

        self.clear_display()

    def _clear_display(self):
        self._oled.fill(0)

    def _show(self):
        self._oled.show()

    def clear_display(self):
        self._clear_display()
        self._show()

    def display_string(self, text):
        self._clear_display()

        self._oled.text(text, 0, OledScreen.TOP_BORDER, 1)
        self._show()

    def display_lines(self, lines):
        self.display_string("\n".join(lines))

    def display_image(self, img_name):
        filePath = IMAGES_PATH + img_name
        image = Image.open(filePath).convert('1')

        self._clear_display()
        self._oled.image(image)
        self._show()