import math

TABLE_H = 3000          # hauteur de table (selon y) (en vrai c'est inversé mais pas envie de casser la simu)
TABLE_W = 2000          # largeur de table (selon x)

# Offset with respect to the center of the robot (if the lidar is not centered)
LIDAR_OFFSET_X = 0
LIDAR_OFFSET_Y = 0

# Max distance between two *successive* measurements for them to belong to the same cluster
CLUSTER_DIST_LIM = 100 # mm

# Ignore detections closer than this
MIN_RANGE = 50 # mm

# Ignore detections further than this
MAX_RANGE = 3000 # mm

# Number of points in a cluster for it to be valid
DETECTION_THRESHOLD = 10

# Coordinates of the three marks
MARK_1_X = 50
MARK_1_Y = -50

MARK_2_X = 1950
MARK_2_Y = -50

MARK_3_X = 1000
MARK_3_Y = 3050

# Uncertainty on the distance between the marks
EPS = 150