@dataclass(frozen=True)
class ObstacleBB:

    BYTE_ORDER = "!"
    LEN_FORMAT = BYTE_ORDER + "i"

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
    def parse(self, message):
        tag, top_x, top_y, width, height = struct.unpack(ObstacleBB.FORMAT, message)
        
        if tag == ObstacleBB._ARUCO_ROBOT:
            return None # Ignore our position; the camera is not accurate enough (let alone fast enough) to beat the BR encoders

        if tag in (ObstacleBB._ARUCO_SUPERSTAR, ObstacleBB._ARUCO_GROUPIE):
            id_ = ObstacleBB.ID_PAMI
        elif tag == ObstacleBB._ARUCO_OPPONENT:
            id_ = ObstacleBB.ID_OPPONENT
        else:
            id_ = ObstacleBB.ID_OTHER

        return ObstacleBB(id_, top_x, top_y, width, height)

    @staticmethod
    def parseList(self, message):
        count, = struct.unpack(ObstacleBB.LEN_FORMAT, message[:4])
        if count != (len(message) - 4) / ObstacleBB.BYTE_SIZE:
            raise ValueError("invalid size")

        obstacles = []
        pos = 4
        for i in range(count):
            obs = ObstacleBB.parse(message[pos:pos+ObstacleBB.BYTE_SIZE])
            pos += ObstacleBB.BYTE_SIZE

            if obs is not None:
                obstacles.append(obs)

        return obstacles
