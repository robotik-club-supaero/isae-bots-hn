class RobotConfig:

    ROBOT_LARG = 240
    ROBOT_LONG = 200

    @property
    def robot_name(self):
        """
        DEPRECATED: No longer used since there is only one robot
        """
        return "R1"

    @property
    def robot_width(self):
        return RobotConfig.ROBOT_LARG

    @property
    def robot_length(self):
        return RobotConfig.ROBOT_LONG
