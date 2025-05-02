# Obstacle detected by a proximity sensor (lidar or sonar especially)

# Set field `dist`  to `DIST_UNKNOWN` (which is also the default value) when the sensor does not support shell-to-shell distance estimation.
float32 DIST_UNKNOWN = -1

# Coordinates of the obstacle. The coordinates are relative to the center of the robot, unless stated otherwise.
# Those are the coordinates of the detected point.
# For the lidar, those are likely to be the coordinates of the opponent's lidar (~ the center of the robot).
float32 x
float32 y

# Distance between the obstacle and the robot shell. May not be set by all the sensors.
float32 dist -1
