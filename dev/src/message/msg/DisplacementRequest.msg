# Displacement request from act_node to disp_node.
# This is NOT what is sent to the BR. See br_messages/DisplacementOrder.

int16 MOVE = 1
# Must be used along with flag MOVE.
int16 BACKWARD = 2
int16 ORIENTATION = 4

# Disables path finding and BR curves.
int16 STRAIGHT_ONLY = 8

# Uses speed control instead of position control
# In this case, x and y are intepreted as percentage of the max linear and angular speeds (between -1 and 1)
# Note that this kind of displacement never stops automatically, you must send a stop order manually.
int16 SPEED_CONTROL = 16

# Destination kind
# FORWARD (1), BACKWARD (3), ORIENTATION (4), STRAIGHT_ONLY (8)
# or combination (FORWARD + ORIENTATION = 5, BACKWARD + ORIENTATION = 7, ...)
# or SPEED_CONTROL (16) (must be used alone)
# or 0 = STOP (must be used alone)
int32 kind

# Destination coordinates
# x, y are ignored unless flag MOVE or SPEED_CONTROL is set in "kind"
# theta is ignored unless flag ORIENTATION is set in "kind"
float32 x
float32 y
float32 theta