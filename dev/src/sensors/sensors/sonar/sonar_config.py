# Position and orientation of the sonars (relative to the robot)
# NOTE: You will need to change the type of topic "/ultrasonicDistances" if there are more than 2 sonars
# (see FIXME notes in sonar_node.py; you'll also need to change the BN)
SONARS = [
    # (x, y, dir_visee), # Sonar 1
    # (x, y, dir_visee), # Sonar 2
    # ...
]

AREA_MARGIN = 150  # actual search area in the table (apply margins)
DIST_MARGIN = 350  # threshold of detection


# Whether to ignore points outside the table
DROP_OFF_LIMITS = True
