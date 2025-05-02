# Displacement request from act_node to disp_node.
# This is NOT what is sent to the BR. See br_messages/DisplacementOrder.

int16 MOVE = 1
# Must be used along with flag MOVE.
int16 BACKWARD = 2
int16 ORIENTATION = 4

# Disables path finding and BR curves.
int16 STRAIGHT_ONLY = 8

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