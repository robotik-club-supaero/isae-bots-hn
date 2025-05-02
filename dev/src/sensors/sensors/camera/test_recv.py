import logging
import time

from .lora import Lora

if __name__ == "__main__":
    lora = Lora(logging.getLogger())
    while True:
        _ = lora.receive()
        time.sleep(0.1)