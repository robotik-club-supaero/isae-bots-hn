from dataclasses import dataclass
from typing import List
import struct
import traceback

from raspi_lora import LoRa, ModemConfig

BYTE_ORDER = "!"

class LoraReceiver:
    INT_FMT = BYTE_ORDER + "i"

    def __init__(self, logger, address, interrupt, spi_channel=0):
        self.logger = logger

        self._lora = LoRa(channel=spi_channel, interrupt=interrupt, this_address=address, acks=False)
        self._lora.on_recv = self._on_recv
        self._lora.set_mode_rx()

    def setCallback(self, callback):
        self._process_obstacles = callback

    def close(self):
        self._lora.close()

    def _process_obstacles(self, obstacles):
        pass
    
    def _on_recv(self, payload):
        self.logger.debug(f"Received LoRa message from {payload.header_from}: {payload.message}")
        if payload.rssi <= -120 or payload.snr <= -13: # todo check if raspi_lora uses db
            self.logger.warn(f"Bad radio link metrics for received message: RSSI: {payload.rssi}, SNR: {payloar.snr}")

        try:
            count, = struct.unpack(LoraReceiver.INT_FMT, message[:4])
            if count != (len(message) - 4) / ObstacleBB.BYTE_SIZE:
                raise ValueError("invalid size")

            message = payload.message
            
            obstacles = []
            pos = 4
            for i in range(count):
                obs = ObstacleBB.parse(message[pos:pos+ObstacleBB.BYTE_SIZE])
                pos += ObstacleBB.BYTE_SIZE

                if obs is not None:
                    obstacles.append(obs)

            self._process_obstacles(obstacles)

        except Exception as e:
            traceback.print_exc()
            self.logger.error(f"Could not parse LoRa message from the camera: {e}")


@dataclass(frozen=True)
class ObstacleBB:
    FORMAT = BYTE_ORDER + "5i"
    BYTE_SIZE = struct.calcsize(FORMAT)

    _ARUCO_SUPERSTAR = 1
    _ARUCO_GROUPIE = 2
    _ARUCO_ROBOT = 3
    _ARUCO_OPPONENT = 51

    ID_OTHER = 0
    ID_PAMI = 1
    ID_OPPONENT = 2

    id: int
    top_x: float
    top_y: float
    width: float
    height: float

    @staticmethod
    def parse(self, payload):
        tag, top_x, top_y, width, height = struct.unpack(ObstacleBB.FORMAT, payload)
        
        if tag == ObstacleBB._ARUCO_ROBOT:
            return None # Ignore our position; the camera is not accurate enough (let alone fast enough) to beat the BR encoders

        if tag in (ObstacleBB._ARUCO_SUPERSTAR, ObstacleBB._ARUCO_GROUPIE):
            id_ = ObstacleBB.ID_PAMI
        elif tag == ObstacleBB._ARUCO_OPPONENT:
            id_ = ObstacleBB.ID_OPPONENT
        else:
            id_ = ObstacleBB.ID_OTHER

        return ObstacleBB(id_, top_x, top_y, width, height)

if __name__ == "__main__":
    import logging
    LoraReceiver(logging.getLogger(), 2, 7)