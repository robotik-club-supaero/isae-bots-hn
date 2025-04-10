from .pathfinder import PathFinderConfig
from .robot import RobotConfig
from .sonar import SonarConfig
from .strat import StratConfig

class GlobalConfig(PathFinderConfig, RobotConfig, SonarConfig, StratConfig):
    pass

__all__ = ["GlobalConfig", "PathFinderConfig", "RobotConfig", "SonarConfig", "StratConfig"]