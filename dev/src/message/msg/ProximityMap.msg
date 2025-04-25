int16 SIM_OBS_NODE = -1
int16 LIDAR = 0
int16 SONAR = 1

# Which sensor does the map come from?
int16 source

# Distance of each cluster to the robot
float32[] cluster_dists

# Relative coordinates of each point (relative to the center of the robot)
float32[] x_r
float32[] y_r

# Cluster index of each point - that's the index in cluster_dists
# This is not a tracking id and will not be consistent accross messages
int32[] cluster