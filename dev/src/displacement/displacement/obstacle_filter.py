import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum

from br_messages.msg import Point, Position

from message.msg import SensorObstacle
from message_utils.geometry import make_relative

@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    static: bool

class RobotSide(IntEnum):
    FRONT = 0
    LEFT = 1
    BACK = 2
    RIGHT = 3

class _ObstacleFilter(ABC):
    """
    Classe de base qui implémente une logique pour filtrer les obstacles en fonction de la direction de déplacement du robot
    """

    def __init__(self, logger, config):
        
        self.robot_width = config.robot_width / 2 # Size along X
        self.robot_length = config.robot_length / 2 # Size along Y
        self.robot_diag = config.robot_diagonal / 2
        self.logger = logger

    def _isRelevant(self, x_r, y_r, backward, any_dir, check_sides=True):
        return any_dir or (not backward and x_r > 0) or (backward and x_r < 0) or \
                (check_sides and abs(x_r) < self.robot_width and abs(y_r) < self.robot_diag) or \
                (check_sides and abs(x_r) < self.robot_diag and abs(y_r) < self.robot_length)

    def _computeDistance(self, obs):
        return NotImplementedError("This sensor should have set field `dist`.")

    @abstractmethod
    def _getObstacles(self):
        pass

    @property
    @abstractmethod
    def _isStatic(self):
        pass

    def _getObstacleDistance(self, obs):
        if obs.dist >= 0:
            return obs.dist
        else:
            try:
                return self._computeDistance(obs)
            except NotImplementedError:
                return max(0, math.sqrt(obs.x*obs.x + obs.y*obs.y) - self.robot_diag)

    def findObstacles(self, backward, any_dir=False, check_sides=True):
        for obs in self._getObstacles():
            if self._isRelevant(obs.x, obs.y, backward, any_dir, check_sides):
                d = self._getObstacleDistance(obs)
                yield Obstacle(obs.x, obs.y, self._isStatic), d

    def findNearestObstacle(self, backward, any_dir=False, check_sides=True, low_limit=0):
        nearest = None
        min_dist = float("inf")
        for obs, dist in self.findObstacles(backward, any_dir, check_sides):
            if dist < min_dist:
                nearest = obs
                min_dist = dist
                
                if min_dist < low_limit:
                    break

        return nearest, min_dist
    
    @property
    def allowUnsafeApproach(self):
        """Indique si le robot est autorisé à s'approcher de l'obstacle plus près que ce qui est normalement autorisé.
        Utilisé pour autoriser le robot à s'approcher des murs
        """
        return False

class ObstacleWalls(_ObstacleFilter):
    """
    Permet de détecter les murs comme des obstacles.
    Ils sont déjà ajoutés comme des obstcles statiques pour le pathfinder, mais certaines actions peuvent nécessiter de
    s'approcher très près des murs en court-circuitant le pathfinder.

    Lors d'un déplacement "straight only", il est de la responsabilité de la stratégie de ne pas demander de pénétrer dans le mur.
    Cette class permet surtout d'éviter de faire une rotation trop près du mur.
    """
 
    def __init__(self, logger, detection_range, config):
        super().__init__(logger, config)

        self.robot_pos = Position()
        self.detection_range = detection_range

    def setRobotPosition(self, position):
        self.robot_pos = position

    @property
    def _isStatic(self):
        return True

    def _getObstacles(self):
        #TOP RIGHT VERTEX:
        #Top_Right.x = center.x + ((width / 2) * cos(angle)) - ((height / 2) * sin(angle))
        #Top_Right.y = center.y + ((width / 2) * sin(angle)) + ((height / 2) * cos(angle))

        #TOP LEFT VERTEX:
        #Top_Left.x = center.x - ((width / 2) * cos(angle)) - ((height / 2) * sin(angle))
        #Top_Left.y = center.y - ((width / 2) * sin(angle)) + ((height / 2) * cos(angle))

        #BOTTOM LEFT VERTEX:
        #Bot_Left.x = center.x - ((width / 2) * cos(angle)) + ((height / 2) * sin(angle))
        #Bot_Left.y = center.y - ((width / 2) * sin(angle)) - ((height / 2) * cos(angle))

        #BOTTOM RIGHT VERTEX:
        #Bot_Right.x = center.x + ((width / 2) * cos(angle)) + ((height / 2) * sin(angle))
        #Bot_Right.y = center.y + ((width / 2) * sin(angle)) - ((height / 2) * cos(angle))

        cos = math.cos(self.robot_pos.theta)
        sin = math.sin(self.robot_pos.theta)

        for i in [1, -1]:
            for j in [1, -1]:
                x_vertex = self.robot_pos.x + i * self.robot_width * cos - j * self.robot_length * sin
                y_vertex = self.robot_pos.y + i * self.robot_width * sin + j * self.robot_length * cos

                if x_vertex < self.detection_range:
                    yield self._makeObstacle(x_vertex, y_vertex, dist=x_vertex)
                if 2000 - x_vertex < self.detection_range:
                    yield self._makeObstacle(x_vertex, y_vertex, dist=2000 - x_vertex)

                if y_vertex < self.detection_range:
                    yield self._makeObstacle(x_vertex, y_vertex, dist=y_vertex)
                if 3000 - y_vertex < self.detection_range:
                    yield self._makeObstacle(x_vertex, y_vertex, dist=3000 - y_vertex)
 
    def _makeObstacle(self, x_vertex, y_vertex, dist):
        x_r, y_r = make_relative(self.robot_pos, Point(x=x_vertex, y=y_vertex))
        return SensorObstacle(x=x_r, y=y_r, dist=abs(dist))

    @property
    def allowUnsafeApproach(self):
        return True

    
class ObstacleNonBypassable(_ObstacleFilter):
    """
    Obstacles qui peuvent être utilisés seulement pour éviter une collision et qui ne peuvent pas
    être pris en compte dans la recherche d'un chemin (sonar)
    """
    
    def __init__(self, logger, config):
        super().__init__(logger, config)

        self.obstacles_sonar = []

    def setObstaclesSonar(self, obstacles):
        self.obstacles_sonar = obstacles.obstacles

    @property
    def _isStatic(self):
        return False

    def _getObstacles(self):
        return iter(self.obstacles_sonar)

        
class ObstacleBypassable(_ObstacleFilter):
    """
    Obstacles qui peuvent être utilisés dans le calcul d'un chemin d'évitement (lidar, caméra, ...)
    """

    def __init__(self, logger, config):
        super().__init__(logger, config)
        self.obstacle_radius = self.robot_diag

        self.obstacles_lidar = []
        # TODO: obstacles camera

    def setObstaclesLidar(self, obstacles):
        self.obstacles_lidar = obstacles.obstacles

    @property
    def _isStatic(self):
        return False

    def _computeDistance(self, obs):
        x_r, y_r = obs.x, obs.y

        # TODO: if we know the orientation of the opponent (thanks to the camera) we can have a better margin
        obstacle_margin = self.obstacle_radius

        # Première estimation, souvent pessimiste (correspond au pire cas, atteint si les robots sont coin-à-coin)
        estimation_1 = math.sqrt(x_r*x_r + y_r*y_r) - obstacle_margin - self.robot_diag

        # On essaie d'avoir une meilleure estimation, en déterminant de quel côté est le robot adverse
        # Pour cela, on détermine le côté du robot qui intersecte la droite qui relie le centre des deux robots
        if x_r > 0:
            if y_r > 0:
                if cross_product_sign(x_r, y_r, self.robot_width, self.robot_length) > 0:
                    side = RobotSide.FRONT
                else:
                    side = RobotSide.LEFT
            else:
                if cross_product_sign(x_r, y_r, self.robot_width, -self.robot_length) > 0:
                    side = RobotSide.RIGHT
                else:
                    side = RobotSide.FRONT
        else:
            if y_r > 0:
                if cross_product_sign(x_r, y_r, -self.robot_width, self.robot_length) > 0:
                    side = RobotSide.LEFT
                else:
                    side = RobotSide.BACK
            else:
                if cross_product_sign(x_r, y_r, -self.robot_width, -self.robot_length) > 0:
                    side = RobotSide.BACK
                else:
                    side = RobotSide.RIGHT

        if side == RobotSide.FRONT or side == RobotSide.BACK:
            # Le robot est devant ou derrière (selon X dans le repère local)
            estimation_2 = abs(x_r) - self.robot_width - obstacle_margin
        else:
            # Le robot est à gauche ou à drotie (selon Y dans le repère local)
            estimation_2 = abs(y_r) - self.robot_length - obstacle_margin

        # Les deux estimations sont pessimistes, donc on prend la plus grande des deux
        # L'estimation 1 a tendance à être plus précise quand les robots sont coin-à-coin (en diagonale),
        # et l'estimation 2 a tendance à être plus précise quand les robots sont côte-à-côte.
        return max(0, estimation_1, estimation_2)

    def _getObstacles(self):
        return iter(self.obstacles_lidar)

def cross_product_sign(x1, y1, x2, y2):
    return x1*y2-y1*x2
