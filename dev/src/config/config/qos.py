from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

default_profile = QoSProfile(depth=10)

latch_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

best_effort_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

br_position_topic_profile = best_effort_profile