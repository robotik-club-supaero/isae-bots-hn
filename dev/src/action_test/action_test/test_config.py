# Cursor orders
CURSOR_UP   = 0
CURSOR_DOWN = 1

# Drawbridge orders
DB_STORE   = 0
DB_PICKUP  = 1
DB_DEPOSIT = 2

# Test sequence: list of (actuator, order) tuples
# actuator: 'cursor' or 'drawbridge'
TEST_ACTIONS = [
    ('cursor',     CURSOR_DOWN),
    ('cursor',     CURSOR_UP),
    ('drawbridge', DB_PICKUP),
    ('drawbridge', DB_DEPOSIT),
    ('drawbridge', DB_STORE),
]
