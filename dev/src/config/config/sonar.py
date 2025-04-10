class SonarConfig:

    """
    SONARS = [
        [x, y, dir_visee]
    ]
    """
    SONARS = []

    @property
    def available_sonars(self):
        return SonarConfig.SONARS