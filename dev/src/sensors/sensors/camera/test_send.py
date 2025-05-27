import logging
import time

from lora import Lora

if __name__ == "__main__":
    lora = Lora(logging.getLogger())
    while True:
        lora.send("TEST".encode("UTF-8"))
        time.sleep(0.1)