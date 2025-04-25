from dataclasses import dataclass

from pathfinder import ObstacleRect

from .robot import RobotConfig

@dataclass(frozen=True)
class StaticPos:
    """Position that is the same for both teams"""
    x: float
    y: float
    theta: float | None = None

    def resolve(self, color=None):
        if self.theta is None:
            return self.x, self.y
        else:
            return self.x, self.y, self.theta

@dataclass(frozen=True)
class DynamicPos:
    """Position that changes depending on the color of the robot"""
    x: float
    y: float
    theta: float | None = None

    def resolve(self, color):
        if color == 0:
            return StaticPos(self.x, self.y, self.theta).resolve()
        else:
            return StaticPos(self.x, 3000-self.y, -self.theta if self.theta is not None else None).resolve()

class NaiveStratConfig(RobotConfig):

    STATIC_OBSTACLES = True # DOIT ETRE 1 EN MATCH REEL

    MATCH_TIME = 100 # s
    DELAY_PARK = 13 # s

    STRAT_NAMES = ['match_strat', 'homologation', 'test_strat']
    DEFAULT_STRAT_INDEX = 0

    @property
    def enable_static_obstacles(self):
        return NaiveStratConfig.STATIC_OBSTACLES

    @property
    def strat_names(self):
        return NaiveStratConfig.STRAT_NAMES

    @property
    def default_strat_index(self):
        return NaiveStratConfig.DEFAULT_STRAT_INDEX

    @property
    def match_time(self):
        return NaiveStratConfig.MATCH_TIME

    @property
    def delay_park(self):
        return NaiveStratConfig.DELAY_PARK
    
class StratConfig(NaiveStratConfig):

    DEFAULT_INIT_ZONE = 0

    # Explication coord :
    # Sur le plan des règles (origine en bas à gauche) : (x, y, theta)
    # Dans notre repère (origine en haut à gauche orienté vers le bas) : (x <= 2000 - y, x <= y, theta <= theta)

    PARK_ZONE = DynamicPos(2000 - 1775, 375, 3.14)

    INIT_ZONES = [
        DynamicPos(2000 - 1825, 375, 0), # 0
        DynamicPos(2000 - 175, 1225  , 3.14),    # 1
        DynamicPos(2000 - 875, 2875, -1.57), # 2
    ]

    PICKUP_STAND_POS = [
        # Protected pickup zone :
        DynamicPos(2000 - 1500, 825, 3.14), 
        # Side Pickup zones :
        StaticPos(2000 - 400, 300, -1.57), StaticPos(2000 - 1325, 300, -1.57),
        StaticPos(2000 - 400, 2700, 1.57), StaticPos(2000 - 1325, 2700, 1.57),
        StaticPos(2000 - 500, 775, 0), StaticPos(2000 - 500, 2225, 0),
        # Middle Pickup zones :
        StaticPos(2000 - 700, 1100, 3.14), StaticPos(2000 - 700, 1900, 3.14) # For start from 0
        #[2000 - 1200, 1100, 0], [2000 - 1200, 1900, 0] # For start from 1, 2
    ]

    DEPOSIT_POS = [
        DynamicPos(2000 - 225, 775, 0), 
        DynamicPos(2000 - 350, 1225, 0), 
        DynamicPos(2000 - 225, 2775, 0), 
        DynamicPos(2000 - 875, 2675, 1.57)
    ]

    def __init__(self, color):
        self.color = color

    @property
    def init_zones(self):
        return self._resolve_pos(StratConfig.INIT_ZONES)
    
    @property
    def default_init_zone_index(self):
        return StratConfig.DEFAULT_INIT_ZONE

    @property
    def pickup_stand_pos(self):
        return self._resolve_pos(StratConfig.PICKUP_STAND_POS)

    @property
    def deposit_pos(self):
        return self._resolve_pos(StratConfig.DEPOSIT_POS)

    @property
    def park_pos(self):
        return self._resolve_pos(StratConfig.PARK_ZONE)

    def static_obstacles(self):
        """Dict of static obstacles for path finder"""

        obstacles = {}
        margin = self.robot_diagonal / 2

        # Walls 
        obstacles["wallNorth"] = ObstacleRect(0, margin, 0, 3000)
        obstacles["wallSouth"] = ObstacleRect(2000-margin, 2000, 0, 3000)
        obstacles["wallEast"] = ObstacleRect(0, 2000, 0, margin)
        obstacles["wallWest"] = ObstacleRect(0, 2000, 3000-margin, 3000)

        # Protected/reserved zones
        # TODO

        # Cans
        # TODO

        return obstacles

    def _resolve_pos(self, pos):
        if isinstance(pos, list):
            return [p.resolve(self.color) for p in pos]
        else:
            return pos.resolve(self.color)