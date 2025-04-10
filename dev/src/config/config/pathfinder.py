class PathFinderConfig:

    STATIC_OBSTACLES = True # DOIT ETRE 1 EN MATCH REEL

    MAX_ASTAR_TIME = 4
    GRID_INTERVAL = 100

    @property
    def enable_static_obstacles(self):
        return PathFinderConfig.STATIC_OBSTACLES

    @property
    def max_astar_time(self):
        return PathFinderConfig.MAX_ASTAR_TIME

    @property
    def grid_interval(self):
        """
        DEPRECATED: New A* no longer uses a grid but uses a visibility graph instead
        """
        return PathFinderConfig.GRID_INTERVAL