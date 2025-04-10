
class StratConfig:

    MATCH_TIME = 100 # s
    DELAY_PARK = 13 # s

    STRAT_NAMES = ['match_strat', 'homologation', 'test_strat']
    DEFAULT_STRAT_INDEX = 0

    INIT_ZONES = [
        [1000,  300, 1.57079632679],
        [1775,  2700, -1.57079632679],
        [225,  2700, -1.57079632679],
    ]
    DEFAULT_INIT_ZONE = 0

    PICKUP_STAND_POS = [
        [800, 1000],
        [1200, 1000]
    ]

    DEPOSIT_POS = [
        [800, 600],
        [1200, 600]
    ]

    @property
    def strat_names(self):
        return StratConfig.STRAT_NAMES

    @property
    def default_strat_index(self):
        return StratConfig.DEFAULT_STRAT_INDEX

    @property
    def match_time(self):
        return StratConfig.MATCH_TIME

    @property
    def delay_park(self):
        return StratConfig.DELAY_PARK

    @property
    def init_zones(self):
        return StratConfig.INIT_ZONES

    @property
    def default_init_zone(self):
        return StratConfig.DEFAULT_INIT_ZONE

    @property
    def pickup_stand_pos(self):
        return StratConfig.PICKUP_STAND_POS

    @property
    def deposit_pos(self):
        return StratConfig.DEPOSIT_POS

    @property
    def park_pos(self):
        return StratConfig.INIT_ZONES  # TODO