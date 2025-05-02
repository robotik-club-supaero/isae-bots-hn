import math

def make_relative(robot_pos, obstacle_pos):
    cos = math.cos(robot_pos.theta)
    sin = -math.sin(robot_pos.theta)

    x = obstacle_pos.x - robot_pos.x
    y = obstacle_pos.y - robot_pos.y

    return x * cos - y * sin, x * sin + y * cos

def make_absolute(robot_pos, obstacle, *, cos=None, sin=None):
    """cos and sin can be given as keyword args if they were already computed, otherwise they will be computed from `robot_pos.theta`"""

    if cos is None: cos = math.cos(robot_pos.theta)
    if sin is None: sin = math.sin(robot_pos.theta)

    return robot_pos.x + obstacle.x*cos - obstacle.y*sin, robot_pos.y + obstacle.y*cos + obstacle.x*sin