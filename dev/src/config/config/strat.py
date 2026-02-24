from dataclasses import dataclass

from pathfinder import ObstacleRect, ObstacleCircle

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
            return StaticPos(self.x, 3000-self.y, ((self.theta + 3.14) % (2*3.14)) if self.theta is not None else None).resolve()

class NaiveStratConfig(RobotConfig):

    STATIC_OBSTACLES = True # DOIT ETRE 1 EN MATCH REEL

    MATCH_TIME = 100 # s
    DELAY_PARK = 10 # s

    MOVE_CURSOR = True
    CURSOR_DISTANCE = 500 # TODO 100 = 10 cm for now TO BE DETERMINED

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
    def cursor_pos(self):
        return StratConfig.CURSOR_POS
    @property
    def cursor_distance(self):
        return self.CURSOR_DISTANCE
    
    @property
    def init_zone_count(self):
        return len(StratConfig.INIT_ZONES)
    
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

    WAIT_PARK_ZONE = DynamicPos(2000 - 1300, 1000, -1.57)
    PARK_ZONE = DynamicPos(2000 - 1775, 375, 0)
    PARK_ZONE_BLUE = DynamicPos(2000 - 1775, 375, 3.14)

    CURSOR_POS = DynamicPos(2000 - 250, 250, 1.57) # To Set
    
    INIT_ZONES = [PARK_ZONE] # Il peut y avoir plusieurs zone de départ -> on peu choisir sur le master node au démarrage

    PICKUP_POS = [
        (DynamicPos(2000 - 1200, 400, 0), 0),
        (DynamicPos(2000 - 400, 400, 0), 1),
        (DynamicPos(2000 - 400, 1200, 1.57), 2),

        (DynamicPos(2000 - 550, 1150, -1.57), 3),
        (DynamicPos(2000 - 1000, 1150, -1.57), 3),
    ]

    DEPOSIT_POS = [
        DynamicPos(2000 - 800, 350, 0), 
        DynamicPos(2000 - 350, 700, 1.57), 
        DynamicPos(2000 - 600, 900, -1.57), 
        DynamicPos(2000 - 1000, 900, 1.57), 
        DynamicPos(2000 - 1250, 1250, -1.57),
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
    def pickup_boxes_pos(self):
        return self._resolve_boxes_pos(StratConfig.PICKUP_POS)

    @property
    def deposit_zones_pos(self):
        return self._resolve_pos(StratConfig.DEPOSIT_POS)

    @property
    def cursor_pos(self):
        return self._resolve_pos(StratConfig.CURSOR_POS)

    @property
    def park_pos(self):
        return self._resolve_pos(StratConfig.PARK_ZONE)

    @property
    def wait_park_pos(self):
        return self._resolve_pos(StratConfig.WAIT_PARK_ZONE)

    def static_obstacles(self):
        """Dict of static obstacles for path finder"""

        obstacles = {}

        if not self.enable_static_obstacles:
            return obstacles

        margin = self.robot_diagonal / 2

        # Walls 
        obstacles["wallNorth"] = ObstacleRect(0, margin, 0, 3000)
        obstacles["wallSouth"] = ObstacleRect(2000-margin, 2000, 0, 3000)
        obstacles["wallEast"] = ObstacleRect(0, 2000, 0, margin)
        obstacles["wallWest"] = ObstacleRect(0, 2000, 3000-margin, 3000)

        # Protected/reserved zones
        # None

        # elements
        elements_margin = margin / 2
        obstacles["box_1"] = ObstacleRect(1000-elements_margin, 1100+elements_margin, 900-elements_margin, 1300+elements_margin)

        return obstacles

    def _resolve_pos(self, pos):
        if isinstance(pos, list):
            return [p.resolve(self.color) for p in pos]
        else:
            return pos.resolve(self.color)

    def _resolve_boxes_pos(self, pos):
        if isinstance(pos, list):
            return [(p.resolve(self.color), box_id) for p, box_id in pos]
        else:
            return [(pos[0].resolve(self.color), pos[1])]
