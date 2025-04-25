from enum import IntEnum, IntFlag
import time
import math

from br_messages.msg import Position
from message.msg import ProximityMap

from config import RobotConfig
from pathfinder import ObstacleCircle, Point as Point_pf
from .pathfinder import PathFinder, PathNotFoundError

STAND_BEFORE_BYPASS = 2 # seconds
# As the opponent is expected to be moving, it should not stay on the path
# It may be more efficient to wait for it to free the way instead of initiating bypass

SLOWDOWN_RANGE = 700 # mm

BYPASS_RANGE = 300 # mm
STOP_RANGE = 100 # mm
MANOEUVER_MARGIN = 100 # mm

LATERAL_MARGIN = 50 # mm

MIN_SPEED = 0.1 # percentage of max speed of BR
MANOEUVER_SPEED = 0.05 # percentage of max speed of BR

class DisplacementStatus(IntEnum):
    IDLE = 0
    MOVING = 1
    WAITING = 2
    AVOIDANCE_MANOEUVER = 3

class DestinationKind(IntFlag):
    NONE = 0
    POSITION = 1
    ORIENTATION = 2
    POSITION_AND_ORIENTATION = 3

class DisplacementManager:
    def __init__(self, communicator, logger):
        self.logger = logger

        self._status = DisplacementStatus.IDLE
        self._backward = False
        self._manoeuverBackward = False
        self._destination = Position()
        self._dest_kind = DestinationKind.NONE
        self._speed = 1.
        self._wait_start = None

        self._robot_pos = Position()

        self.communicator = communicator

        config = RobotConfig()
        self.robot_diag = config.robot_diagonal / 2

        self.proximity = ProximityObstacles(LATERAL_MARGIN, config)
        self.map = PathFinder({})

    def setRobotPosition(self, position):
        self._robot_pos = position

    def setStaticObstacles(self, obstacles: list):
        self.map = PathFinder(obstacles)

    def getPathFinder(self):
        return self.map

    def reportSensorDetection(self, obstacles: ProximityMap):
        self.proximity.setObstacles(obstacles)

    def clearSensorDetection(self, sensor=None):
        self.proximity.clearObstacles(sensor)

    def cancelDisplacement(self):
        self._stop()
        self._status = DisplacementStatus.IDLE
        self._dest_kind = DestinationKind.NONE

    def requestDisplacementTo(self, destination, backward, final_orientation):
        self._destination.x = destination.x
        self._destination.y = destination.y
        self._backward = backward
        self._dest_kind = DestinationKind.POSITION
        if final_orientation is not None:
            self._destination.theta = final_orientation
            self._dest_kind |= DestinationKind.ORIENTATION
        
        self._resume_move()

    def requestRotation(self, theta):
        self._destination.theta = theta
        self._dest_kind = DestinationKind.ORIENTATION
        self._resume_move()

    def _resume_move(self):
        if self._status != DisplacementStatus.IDLE:
            self._stop()
            
        self._setSpeed(1.)

        if self._dest_kind & DestinationKind.POSITION != 0:
            try:
                path = self.map.get_path([self._robot_pos.x, self._robot_pos.y], [self._destination.x, self._destination.y])
                self._setObstacleToBypass(None)

                self.logger.debug("Found path: " + str(path))

                theta = self._destination.theta if self._dest_kind & DestinationKind.ORIENTATION != 0 else None
                self.communicator.sendPathCommand(path, self._backward, theta, allow_curve=True)
                self._status = DisplacementStatus.MOVING
            except PathNotFoundError:
                self.logger.warn(f"Cannot find a path to ({self._destination.x}, {self._destination.y})")
                self.cancelDisplacement()
                self.communicator.reportPathNotFound()

        elif self._dest_kind & DestinationKind.ORIENTATION != 0:
            self.communicator.sendOrientationCommand(self._destination.theta)
        
        else:
            self.logger.error(f"No destination is set or destination kind is not recognized.")

    def _stop(self):
        self.communicator.sendStopCommand()

    def _stopAndWait(self):
        self._stop()
        self._status = DisplacementStatus.WAITING
        self._wait_start = time.time()

    def _setSpeed(self, speed):
        if abs(speed - self._speed) > 0.01:            
            self._speed = speed
            self.communicator.sendSetSpeed(int(100*speed))

    def _setObstacleToBypass(self, obs):
        NAME = "_dyn_bypass"
        if obs is None:
            self.map.remove_obstacle(NAME)
        else:
            self.map.set_dynamic_obstacle(NAME, ObstacleCircle(Point_pf(*obs), BYPASS_RANGE + self.robot_diag))

    def _startManoeuver(self):
        _, dist_forward = self.proximity.findNearestObstacle(backward=False, check_sides=False, low_limit=STOP_RANGE, high_limit=SLOWDOWN_RANGE)
        _, dist_backward = self.proximity.findNearestObstacle(backward=True, check_sides=False, low_limit=STOP_RANGE, high_limit=SLOWDOWN_RANGE)

        if dist_forward < dist_backward:
            if dist_backward > STOP_RANGE:
                self._manoeuverBackward = True
                self._status = DisplacementStatus.AVOIDANCE_MANOEUVER
            else:
                return
        else:
            if dist_forward > STOP_RANGE:
                self._manoeuverBackward = False
                self._status = DisplacementStatus.AVOIDANCE_MANOEUVER
            else:
                return

        offset_x, offset_y = self._robot_pos.x + math.cos(self._robot_pos.theta), self._robot_pos.y + math.sin(self._robot_pos.theta)

        speed = int(255 * MANOEUVER_SPEED)
        if self._manoeuverBackward:
            speed = -speed
            offset_x = -offset_x
            offset_y = -offset_y

            try:
                _ = self.map.get_path([self._robot_pos.x, self._robot_pos.y], [self._robot_pos.x + offset_x, self._robot_pos.y + offset_y])
            except:
                return # Not enough room for manoeuver (may be too close to a wall or another static obstacle)
        
        self.logger.info("Obstacle too close for bypass. Starting avoidance manoeuver.")
        self.communicator.sendSpeedCommand(speed, 0)

    def update(self):
        if self._status == DisplacementStatus.MOVING:
            _obs, dist = self.proximity.findNearestObstacle(self._backward, low_limit=BYPASS_RANGE, high_limit=SLOWDOWN_RANGE)

            if dist < BYPASS_RANGE:
                self.logger.warn("Obstacle detected: need to wait")
                self._stopAndWait()
            else:
                speed = max(MIN_SPEED, min(1, (dist-STOP_RANGE)/(SLOWDOWN_RANGE-STOP_RANGE)))
                self._setSpeed(speed)

        elif self._status == DisplacementStatus.WAITING:
            obs, dist = self.proximity.findNearestObstacle(self._backward, low_limit=STOP_RANGE, high_limit=SLOWDOWN_RANGE)

            if dist < STOP_RANGE and time.time() - self._wait_start > STAND_BEFORE_BYPASS:
                self._startManoeuver()

            elif dist > BYPASS_RANGE or time.time() - self._wait_start > STAND_BEFORE_BYPASS:
                if dist > BYPASS_RANGE:
                    self.logger.info("Obstacle has cleared the way: resuming displacement")
                else:
                    self.logger.info("Initiating bypass")
                self._setObstacleToBypass(obs)
                self._resume_move()

        elif self._status == DisplacementStatus.AVOIDANCE_MANOEUVER:
            obs, dist = self.proximity.findNearestObstacle(self._backward, low_limit=STOP_RANGE, high_limit=SLOWDOWN_RANGE)
            if dist > BYPASS_RANGE:
                self.logger.info("Manoeuver complete: resuming displacement")
                self._stop()
                self._setObstacleToBypass(obs)
                self._resume_move()

            else:
                _obs, dist = self.proximity.findNearestObstacle(self._manoeuverBackward, check_sides=False, low_limit=STOP_RANGE, high_limit=SLOWDOWN_RANGE)
                if dist < STOP_RANGE:
                    self.logger.info("Obstacle detected too close: aborting manoeuver")
                    self._stopAndWait()

class RobotSide(IntEnum):
    FRONT = 0
    LEFT = 1
    BACK = 2
    RIGHT = 3

class ProximityObstacles:
    def __init__(self, lateral_margin, config):
        self.obstacles = {}

        self.robot_width = config.robot_width / 2
        self.robot_length = config.robot_length / 2
        self.robot_diag = config.robot_diagonal / 2
        self.lateral_margin = lateral_margin

    def _isRelevant(self, x_r, y_r, backward, any_dir, check_sides=True):
        return any_dir or (not backward and x_r > 0) or (backward and x_r < 0) or \
                (check_sides and abs(x_r) < self.robot_width and abs(y_r) < self.robot_width + self.lateral_margin)

    def _computeDistance(self, x_r, y_r):
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
            t = abs(self.robot_width / x_r)
        else:
            t = abs(self.robot_length / y_r)
        t = min(t, 1)

        return math.sqrt(x_r**2+y_r**2) * (1 - t)

    def findNearestObstacle(self, backward=False, any_dir=False, check_sides=True, low_limit=0, high_limit=None):
        nearest = None
        min_dist = float('inf')

        for obstacles in self.obstacles.values():
            skip_cluster = None
            for x_r, y_r, cluster in zip(obstacles.x_r, obstacles.y_r, obstacles.cluster):
                
                if skip_cluster == cluster:
                    continue
                if high_limit is not None and obstacles.cluster_dists[cluster] > high_limit + self.robot_diag:
                    skip_cluster = cluster
                    continue

                if self._isRelevant(x_r, y_r, backward, any_dir, check_sides):
                    d = self._computeDistance(x_r, y_r)

                    if d < min_dist:
                        nearest = (x_r, y_r)
                        min_dist = d

                        if d < low_limit:
                            break
        
        return (nearest, min_dist)
        
    def setObstacles(self, proximity_map):
        # Override the previous obstacles for this sensor only.
        self.obstacles[proximity_map.source] = proximity_map

    def clearObstacles(self, source):
        if source is None:
            self.obstacles.clear()
        else:
            self.obstacles.pop(source, None)

def cross_product_sign(x1, y1, x2, y2):
    return x1*y2-y1*x2
