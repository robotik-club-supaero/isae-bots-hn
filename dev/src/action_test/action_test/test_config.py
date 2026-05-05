from strat.act.an_const import CursorOrder, CursorCallback, DrawbridgeOrder, DrawbridgeCallback

# Test sequence: list of (actuator, order) tuples
# actuator: 'cursor' or 'drawbridge'
TEST_ACTIONS = [ # Name of the actuator (for publisher), order, response expected
    ('cursor',     CursorOrder.DOWN, CursorCallback.DOWN),
    ('drawbridge', DrawbridgeOrder.STORE, DrawbridgeCallback.STORE),
    ('drawbridge', DrawbridgeOrder.PICKUP, DrawbridgeCallback.PICKUP),
    ('drawbridge', DrawbridgeOrder.DEPOSIT, DrawbridgeCallback.DEPOSIT),
    ('drawbridge', DrawbridgeOrder.STORE, DrawbridgeCallback.STORE),
    ('cursor',     CursorOrder.UP, CursorCallback.UP),
]

TEST_TIMEOUT = 10.0 # s
