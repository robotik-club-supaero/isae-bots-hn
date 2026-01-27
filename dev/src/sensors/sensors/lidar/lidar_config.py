import math

TABLE_H = 3000          # hauteur de table (selon y)
TABLE_W = 2000          # largeur de table (selon x)

# Offset with respect to the center of the robot (if the lidar is not centered)
LIDAR_OFFSET_X = 0
LIDAR_OFFSET_Y = 0
LIDAR_ANGLE = 90 # Degrés

# Max distance between two *successive* measurements for them to belong to the same cluster
CLUSTER_DIST_LIM = 100 # mm

# Ignore detections closer than this
MIN_RANGE = 50 # mm

# Ignore detections further than this
MAX_RANGE = 3000 # mm

# Whether to ignore points outside the table
DROP_OFF_LIMITS = True

DETECTION_THRESHOLD = 10