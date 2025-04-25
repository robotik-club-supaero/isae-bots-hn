from pathfinder import Map, Point

class PathNotFoundError(RuntimeError):
    pass

class PathFinder:

    def __init__(self, static_obstacles):
        self.map = Map(static_obstacles)
        
    def set_dynamic_obstacle(self, name, obstacle):
        self.map.setObstacle(name, obstacle)

    def remove_obstacle(self, obstacle_name):
        self.map.setObstacle(obstacle_name, None)

    def get_path(self, init, goal):
        path = self.map.astarPath(Point(*init), Point(*goal))
        if path is None or len(path) == 0:
            raise PathNotFoundError()
        
        return path

    def get_grid(self):
        grid = self.map.getGrid()
        return [[point.x, point.y] for point in grid]