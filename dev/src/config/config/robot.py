import math

class RobotConfig:

    ROBOT_LONG = 200 # Selon X local
    ROBOT_LARG = 240 # Selon Y local

    @property
    def robot_name(self):
        """
        DEPRECATED: No longer used since there is only one robot
        """
        return "R1"

    @property
    def robot_length(self):
        return RobotConfig.ROBOT_LONG

    @property
    def robot_width(self):
        return RobotConfig.ROBOT_LARG

    @property
    def robot_diagonal(self):
        return math.sqrt(self.robot_width**2 + self.robot_length**2)