from .robot import RobotConfig
from .strat import StratConfig, NaiveStratConfig

COLOR = ["HOME", "AWAY"]

__all__ = ["COLOR", "RobotConfig", "StratConfig", "NaiveStratConfig"]
# Here, we make sure that `from config import *` will not import modules `qos`, `robot` and `strat` into the root namespace
# We want the user to use fully-qualified name (`config.qos`) or explicit import alias (`import config.qos as qos_config`) instead
# so there is no ambiguity.