from .robot import RobotConfig
from .sonar import SonarConfig
from .strat import StratConfig, NaiveStratConfig

COLOR = ["HOME", "AWAY"]

__all__ = ["COLOR", "RobotConfig", "SonarConfig", "StratConfig", "NaiveStratConfig"]