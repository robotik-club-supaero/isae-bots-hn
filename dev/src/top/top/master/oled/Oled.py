import os

import board
import busio
import digitalio
import adafruit_ssd1306

from PIL import Image, ImageDraw, ImageFont

IMAGES_PATH = f"{os.path.dirname(os.path.realpath(__file__))}/images/"

class OledScreen:

    WIDTH = 128
    HEIGHT = 64

    def __init__(self, addr=0x3d, oled_reset=None):
        if oled_reset is not None:
            oled_reset = digitalio.DigitalInOut(oled_reset)

        i2c = busio.I2C(board.SCL, board.SDA)
        self._oled = adafruit_ssd1306.SSD1306_I2C(OledScreen.WIDTH, OledScreen.HEIGHT, i2c, addr=addr, reset=oled_reset)

        self._image = Image.new("1", (self._oled.width, self._oled.height))
        self._draw = ImageDraw.Draw(image)
        self._font = ImageFont.load_default()

        self._oled.fill(0)
        self._oled.show()
        
        self.clear_display()

    def _show(self, image=None):
        if image is None: image = self._image
        self._oled.image(image)
        self._oled.show()

    def _clear_image(self):
        oled = self._oled

        # Draw a black background
        self._draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    def clear_display(self):
        self._clear_image()
        self._show()

    def display_string(self, text):
        self._clear_image()

        oled = self._oled

        # Draw Some Text
        bbox = self._font.getbbox(text)
        (font_width, font_height) = bbox[2] - bbox[0], bbox[3] - bbox[1]
        self._draw.text(
            (oled.width // 2 - font_width // 2, oled.height // 2 - font_height // 2),
            text,
            font=self._font,
            fill=255,
        )

        self._show()

    def display_lines(self, lines):
        self.display_string("\n".join(lines))

    def display_image(self, img_name):
        filePath = IMAGES_PATH + fileName
        image = Image.open(filePath).convert('1')

        self._show(image)