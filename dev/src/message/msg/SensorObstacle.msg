# Set field `dist`  to `DIST_UNKNOWN` (which is also the default value) when the sensor does not support shell-to-shell distance estimation.
float32 DIST_UNKNOWN = -1

# Coordinates of the obstacle. Can be absolute or relative to the center of the robot depending on the topic used.
float32 x
float32 y

# Distance between the obstacle and the robot shell. May not be set by all the sensors.
float32 dist -1
