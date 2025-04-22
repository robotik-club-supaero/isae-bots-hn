
class StratConfig:

    MATCH_TIME = 100 # s
    DELAY_PARK = 13 # s

    STRAT_NAMES = ['match_strat', 'homologation', 'test_strat']
    DEFAULT_STRAT_INDEX = 0
    DEFAULT_INIT_ZONE = 0

    # Explication coord :
    # Sur le plan des règles (origine en bas à gauche) : (x, y, theta)
    # Dans notre repère (origine en haut à gauche orienté vers le bas) : (x <= 2000 - y, x <= y, theta <= theta)

    PARK_ZONE = [2000 - 1775, 375, 3.14]

    INIT_ZONES = [
        [2000 - 1825, 375, 0], # 0
        [2000 - 175, 1225  , 3.14],    # 1
        [2000 - 875, 2875, -1.57],# 2
    ]

    PICKUP_STAND_POS = [
        # Protected pickup zone :
        [2000 - 1500, 825, 3.14], 
        # Side Pickup zones :
        [2000 - 400, 300, -1.57], [2000 - 1325, 300, -1.57],
        [2000 - 400, 2700, 1.57], [2000 - 1325, 2700, 1.57],
        [2000 - 500, 775, 0], [2000 - 500, 2225, 0],
        # Middle Pickup zones :
        [2000 - 700, 1100, 3.14], [2000 - 700, 1900, 3.14] # For start from 0
        #[2000 - 1200, 1100, 0], [2000 - 1200, 1900, 0] # For start from 1, 2
    ]

    DEPOSIT_POS = [
        [2000 - 225, 775, 0], 
        [2000 - 350, 1225, 0], 
        [2000 - 225, 2775, 0], 
        [2000 - 875, 2675, 1.57]
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
        return StratConfig.PARK_ZONE
