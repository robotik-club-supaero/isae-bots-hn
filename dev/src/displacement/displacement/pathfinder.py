from pathfinder import Map, Point

USE_REGULAR_GRID = False
GRID_INTERVAL = 50 # mm

class PathNotFoundError(RuntimeError):
    pass

class PathFinder:

    def __init__(self, static_obstacles):
        self.map = Map(static_obstacles)
        if USE_REGULAR_GRID:
            self.map.switchToRegularGrid(0, 2000, 0, 3000, GRID_INTERVAL)
        # else:
            # The default is a visibility graph

        self.dyn_obstacles = set()
        
    def set_dynamic_obstacle(self, name, obstacle):
        if obstacle is not None:
            self.dyn_obstacles.add(name)

        self._set_obstacle(name, obstacle)

    def _set_obstacle(self, name, obstacle):
        if obstacle is None:
            try:
                self.dyn_obstacles.remove(name)
            except KeyError:
                pass

        self.map.setObstacle(name, obstacle)

    def set_dynamics_obstacles(self, obstacles):
        to_remove = self.dyn_obstacles.copy()
        for name, obstacle in obstacles.items():
            to_remove.remove(name)
            self.set_dynamic_obstacle(name, obstacle)
        
        for name in to_remove:
            self.remove_obstacle(name)

    def clear_dynamic_obstacles(self):
        for name in self.dyn_obstacles:
            self.remove_obstacle(name)

    def remove_obstacle(self, obstacle_name):
        self._set_obstacle(obstacle_name, None)

    def get_obstacle(self, obstacle_name):
        return self.map.getObstacle(obstacle_name)

    def can_go_straight(self, init, goal):
        return self.map.canGoStraight(Point(*init), Point(*goal))

    def get_path(self, init, goal):
        path = self.map.astarPath(Point(*init), Point(*goal))
        if path is None or len(path) == 0:
            raise PathNotFoundError()
        
        return path

    def get_grid(self):
        grid = self.map.getGrid()
        return [[point.x, point.y] for point in grid]