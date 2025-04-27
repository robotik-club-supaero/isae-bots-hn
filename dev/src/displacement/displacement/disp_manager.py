from enum import IntEnum, IntFlag
import time
import math

from br_messages.msg import Position

from config import RobotConfig
from pathfinder import ObstacleCircle, Point as Point_pf
from message_utils.geometry import make_absolute

from .obstacle_filter import ObstacleBypassable, ObstacleNonBypassable, ObstacleWalls
from .pathfinder import PathFinder, PathNotFoundError

STAND_BEFORE_BYPASS = 2 # seconds
# As the opponent is expected to be moving, it should not stay on the path
# It may be more efficient to wait for it to free the way instead of initiating bypass

BLOCKED_TIMEOUT = 2 # seconds

SLOWDOWN_RANGE = 700 # mm
# Distance en dessous de laquelle le robot va ralentir pour "laisse passer" l'obstacle et, avec un peu de chance,
# ne pas avoir besoin de s'arrêter après

DYNAMIC_VISIBILITY = 500 # mm
# Distance au-delà de laquelle les obstacles dynamiques sont ignorés par le PF
# Leur position actuelle ne sera probablement plus valide d'ici là.

BYPASS_RANGE = 300 # mm
# Distance en dessous de laquelle le robot va s'arrêter et recalculer un chemin de contournement

STOP_RANGE = 50 # mm
# Distance critique en dessous de laquelle le contournement n'est pas possible.
# Des manoeuvres d'éloignement lentes peuvent toujours être tentées.

MANOEUVER_ESCAPE_THRESHOLD= 150 # mm
# Distance à partir de laquelle une manoeuvre d'éloignement est considérée comme terminée

MIN_SPEED = 0.4 # percentage of max speed of BR
MANOEUVER_SPEED = 0.05 # percentage of max speed of BR

BYPASS_OBSTACLE_NAME = "_dyn_bypass"

class DisplacementStatus(IntEnum):
    IDLE = 0
    MOVING = 1
    WAITING = 2
    AVOIDANCE_MANOEUVER = 3

class DisplacementManager:
    def __init__(self, communicator, logger):
        self.logger = logger

        self._status = DisplacementStatus.IDLE

        self._destination_pos = None
        self._destination_theta = None
        self._backward = False
        self._straight_only = False

        self._manoeuverBackward = False

        self._speed = 1.
        self._wait_start = None
        self._bypassing = False
        self._blocked = False
        self._manoeuverBlocked = False

        self._robot_pos = Position()

        self.communicator = communicator

        config = RobotConfig()
        self.robot_diag = config.robot_diagonal / 2
        self.obstacle_radius = self.robot_diag

        self.obstacles_bypassable = ObstacleBypassable(logger, config)
        self.obstacles_non_bypassable = ObstacleNonBypassable(logger, config)
        self.obstacles_wall = ObstacleWalls(logger, STOP_RANGE, config)
        self._enable_wall_detection = True

        self.map = PathFinder({})

    def setRobotPosition(self, position):
        self._robot_pos = position
        self.obstacles_wall.setRobotPosition(position)
        self.update()

    def setStaticObstacles(self, obstacles):
        self.map = PathFinder(obstacles)

    def getPathFinder(self):
        return self.map

    def setLidarObstacles(self, obstacles):
        self.obstacles_bypassable.setObstaclesLidar(obstacles)

    def setSonarObstacles(self, obstacles):
        self.obstacles_non_bypassable.setObstaclesSonar(obstacles)

    def _clearState(self):
        self._status = DisplacementStatus.IDLE
        self._destination_pos = None
        self._destination_theta = None
        self._bypassing = False
        self._blocked = False
        self._manoeuverBlocked = False 

    def cancelDisplacement(self):
        self._stop()
        self._clearState()

    def requestDisplacementTo(self, destination, backward, final_orientation, straight_only=False):
        self._clearState()
        self._destination_pos = destination
        self._destination_theta = final_orientation
        self._backward = backward
        self._straight_only = straight_only
        self._resume_move()

    def requestRotation(self, theta):
        self._clearState()
        self._destination_pos = None
        self._destination_theta = theta
        self._resume_move()

    def _resume_move(self):

        self._setSpeed(1.)

        if self._destination_pos is not None:
            try:
                if self._straight_only:
                    path = [self._destination_pos]
                else:
                    self._updateObstacleToBypass()
                    path = self._getPathToDest()
                    self.logger.debug("Found path: " + str(path))

                self._blocked = False
                self._manoeuverBlocked = False
                self.communicator.sendPathCommand(path, self._backward, self._destination_theta , allow_curve=not self._straight_only)
                self._status = DisplacementStatus.MOVING

            except PathNotFoundError:
                if self._bypassing:
                    if self._blocked:
                        if time.time() - self._wait_start > BLOCKED_TIMEOUT:
                            self.logger.warn(f"Displacement aborted because the destination has been blocked for too long.")
                            self.cancelDisplacement()
                            self.communicator.reportPathNotFound()
                    else:
                        self._blocked = True
                        self.logger.warn(f"Cannot bypass because the destination is blocked by the opponent.")
                        self._stopAndWait()
                else:
                    self.logger.warn(f"Cannot find a path to ({self._destination_pos.x}, {self._destination_pos.y})")
                    self._stopAndWait()

        elif self._destination_theta is not None:
            self.communicator.sendOrientationCommand(self._destination_theta)
        
        else:
            self.logger.error(f"No destination is set or destination kind is not recognized.")

    def _getPathToDest(self):
        return self.map.get_path([self._robot_pos.x, self._robot_pos.y], [self._destination_pos.x, self._destination_pos.y])

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

    def _clearObstacleToBypass(self):
        self.map.remove_obstacle(BYPASS_OBSTACLE_NAME)

    def _updateObstacleToBypass(self):      
        obs, dist = self.obstacles_bypassable.findNearestObstacle(self._backward)
        if obs is None or dist > DYNAMIC_VISIBILITY:
            self._bypassing = False
            self._clearObstacleToBypass()
        else:
            x_abs, y_abs = make_absolute(self._robot_pos, obs)
            self.map.set_dynamic_obstacle(BYPASS_OBSTACLE_NAME, ObstacleCircle(Point_pf(x_abs, y_abs), self.robot_diag + self.obstacle_radius + STOP_RANGE))

    def _startManoeuver(self):
        _, dist_forward = self._findNearestObstacle(backward=False, check_sides=False, manoeuver=True)
        _, dist_backward = self._findNearestObstacle(backward=True, check_sides=False, manoeuver=True)

        if dist_forward < dist_backward:
            if dist_backward > STOP_RANGE:
                self._manoeuverBackward = True
            else:
                return False
        else:
            if dist_forward > STOP_RANGE:
                self._manoeuverBackward = False
            else:
                return False
        
        speed = 255. * MANOEUVER_SPEED
        if self._manoeuverBackward:
            speed = -speed

        self._status = DisplacementStatus.AVOIDANCE_MANOEUVER
        self.logger.info("Obstacle too close for bypass. Starting avoidance manoeuver.")
        self.communicator.sendSpeedCommand(speed, 0.)
        return True

    def update(self):
        if self._status == DisplacementStatus.MOVING:
            _obs, dist = self._findNearestObstacle(self._backward)

            if dist < STOP_RANGE:
                self.logger.warn("Obstacle detected too close: need to wait")
                self._stopAndWait()
            else:
                if not self._bypassing and not self._straight_only and dist < BYPASS_RANGE:
                    try:
                        # Check if the opponent is on the way and is a "bypassable" obstacle:
                        # - compute the path without the obstacle
                        # - add the obstacle (if it is a bypassable obstacle)
                        # - check if the path is still valid

                        self._clearObstacleToBypass()
                        path = self._getPathToDest()
                        self._updateObstacleToBypass()
                        if not self.map.can_go_straight([self._robot_pos.x, self._robot_pos.y], [path[0].x, path[0].y]):
                            # Opponent effectively blocking the way
                            self.logger.warn("Obstacle detected on the way: need to wait")
                            self._stopAndWait()
                            return
                    except:
                        # Should not happen because we could compute the path the first time
                        self.logger.error("Unexpected error when recomputing the path")
                        pass
                
                low_bound = STOP_RANGE if self._bypassing or self._straight_only else BYPASS_RANGE
                speed = max(MIN_SPEED, min(1, (dist-low_bound)/(SLOWDOWN_RANGE-low_bound)))
                self._setSpeed(speed)

        elif self._status == DisplacementStatus.WAITING:
            obs, dist = self._findNearestObstacle(self._backward)

            if dist < STOP_RANGE and (obs.static or time.time() - self._wait_start > STAND_BEFORE_BYPASS):
                if self._manoeuverBlocked:
                    self.logger.warn(f"Displacement aborted because the robot is blocked.")
                    self.cancelDisplacement()
                    self.communicator.reportBlocked()

                elif not self._startManoeuver():
                    self.logger.info("Not enough room to initiate manoeuver")
                    self._manoeuverBlocked = True
                    self._stopAndWait()

            elif dist > BYPASS_RANGE or time.time() - self._wait_start > STAND_BEFORE_BYPASS:
                if not self._blocked:
                    if dist > BYPASS_RANGE:
                        self.logger.info("Obstacle has cleared the way: resuming displacement")
                    else:
                        self.logger.info("Initiating bypass")

                self._bypassing = True
                self._resume_move()

        elif self._status == DisplacementStatus.AVOIDANCE_MANOEUVER:
            obs, dist = self._findNearestObstacle(self._backward)
            if dist > MANOEUVER_ESCAPE_THRESHOLD:                    
                self._updateObstacleToBypass()
                self.logger.info("Manoeuver complete: resuming displacement")
                self._stopAndWait()
                self._bypassing = True
                self._resume_move()

            else:
                _obs, dist = self._findNearestObstacle(self._manoeuverBackward, check_sides=False, manoeuver=True)
                self.logger.info(str(dist))
                if dist < STOP_RANGE:
                    self.logger.info("Obstacle detected too close: aborting manoeuver")
                    self._stopAndWait()

    def _obstacleSources(self):
        yield self.obstacles_non_bypassable
        yield self.obstacles_bypassable
        if self._enable_wall_detection:
            yield self.obstacles_wall

    def _findNearestObstacle(self, backward, any_dir=False, check_sides=True, manoeuver=False, low_limit=STOP_RANGE):
        nearest, min_dist = None, float("inf")
        
        for source in self._obstacleSources():
            obs, dist = source.findNearestObstacle(backward, any_dir, check_sides, low_limit)
            if source.allowUnsafeApproach and (manoeuver or self._straight_only) and dist > 0:
                dist = max(dist, STOP_RANGE)

            if dist < low_limit:
                return obs, dist
            if nearest is None or dist < min_dist:
                nearest = obs
                min_dist = dist

        return nearest, min_dist