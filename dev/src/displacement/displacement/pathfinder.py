import numpy as np
from ast import literal_eval

from pathfinder import Map, ObstacleCircle, ObstacleRect, Point

# from strat.strat_const import PLANTS_POS, POTS_POS
from strat.strat_utils import adapt_pos_to_side
from .disp_utils import *

HOME = 0
AWAY = 1

class PathNotFoundError(RuntimeError):
    pass

class PathFinder:

    def __init__(self, color, logger):
        self.map = Map(PathFinder.static_obstacles(color, logger))
        self._logger = logger

    def set_obstacle_robot_pos(self, obstacle_robot_pos, radius):
        self.map.setObstacle("robot_pos", ObstacleCircle(Point(*obstacle_robot_pos[:2]), radius) if obstacle_robot_pos is not None else None)
    
    def get_obstacle_robot_pos(self):
        obs = self.map.getObstacle("robot_pos")
        if obs is None or not isinstance(obs, ObstacleCircle):
            return None
        return obs.center

    def remove_obstacle(self, obstacle):
        self.map.setObstacle(obstacle, None)

    def get_path(self, init, goal):
        path = self.map.astarPath(Point(*init), Point(*goal))
        if path is None or len(path) == 0:
            raise PathNotFoundError()
        
        return path

    def get_grid(self):
        grid = self.map.getGrid()
        return np.array([[point.x, point.y] for point in grid])

    @staticmethod
    def static_obstacles(color, logger):
        """Fonction retournant un dictionnaire d'obstacles statiques."""

        ## STATIC OBSTACLES ###############################################
        obstacles = {}

        if CONFIG.enable_static_obstacles:

            # Walls 
            obstacles["wallNorth"] = ObstacleRect(0, MARGIN, 0, 3000)
            obstacles["wallSouth"] = ObstacleRect(2000-MARGIN, 2000, 0, 3000)
            obstacles["wallEast"] = ObstacleRect(0, 2000, 0, MARGIN)
            obstacles["wallWest"] = ObstacleRect(0, 2000, 3000-MARGIN, 3000)

            # Bases
            baseHome = ObstacleRect(0, 450+MARGIN, 2550-MARGIN, 3000)
            baseAway = ObstacleRect(0, 450+MARGIN, 0, 450+MARGIN)
            if color == HOME:
                obstacles["oppBase"] = baseAway
            else:
                obstacles["oppBase"] = baseHome

            # Plants # TODO update
            #for i, plant in enumerate(PLANTS_POS):
            #    obstacles[f"plant{i}"] = ObstacleCircle(Point(*adapt_pos_to_side(*plant, 0, color)[:2]), 125+MARGIN)

            # Pots # TODO update
            #for i, pot in enumerate(POTS_POS):
            #    obstacles[f"pot{i}"] = ObstacleCircle(Point(*adapt_pos_to_side(*pot, color)[:2]), 75+MARGIN)

        else:
            logger.warn("Static obstacles have been disabled! Make sure to enable before a real match.")

        logger.info("Number of static obstacles : {}.".format(len(obstacles)))
    
        return obstacles
