int16 MOVE = 1
# MOVE (1) | BACKWARD_FLAG (2) = 3
int16 BACKWARD = 3
int16 ORIENTATION = 4

# Must be used along with flag MOVE. Use constant BACKWARD instead
int16 BACKWARD_FLAG = 2

# Destination kind
# FORWARD (1), BACKWARD (3), ORIENTATION (4) or combination (FORWARD + ORIENTATION = 5, BACKWARD + ORIENTATION = 7)
# or 0 = STOP
int32 kind

# Destination coordinates
# x, y are ignored unless flag MOVE is set in "kind"
# theta is ignored unless flag ORIENTATION is set in "kind"
float32 x
float32 y
float32 theta