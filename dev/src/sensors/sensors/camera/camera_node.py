import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from config.qos import latch_profile
from message.msg import RectObstacle, CameraObstacleList

from .lora import LoraReceiver, ObstacleBB

LORA_ADDRESS = 2
INTERRUPT_PIN = 17

class CameraNode(Node):

    def __init__(self):
        super().__init__("CAM")

        self._lora = LoraReceiver(self.get_logger(), LORA_ADDRESS, INTERRUPT_PIN)
        self._lora.setCallback(self.recv_message)

        self.pub_obstacles = self.create_publisher(Empty, "/sensors/obstaclesCamera", latch_profile)
        self.msg = CameraObstacleList()

    def recv_message(self, obstacles):
        msg = self.msg
        msg.opponent_detected = False
        msg.pamis.clear()
        msg.other_obstacles.clear()

        for obs in obstacles:
            obs_rect = RectObstacle(x=obs.top_x, y=y.top_y, width=obs.width)
            if obs.id == ObstacleBB.ID_OPPONENT:
                msg.opponent_detected = True
                msg.opponent = obs_rect
            elif obs.id == ObstacleBB.ID_PAMI:
                msg.pamis.append(obs_rect)
            else:
                msg.other_obstacles.append(obs_rect)

        self.pub_obstacles.publish(msg)

    def close(self):
        self._lora.close()

    def destroy_node(self):
        self.close()
        super().destroy_node()

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    #############################################################
    # INITIALIZATION
    #############################################################

    rclpy.init(args=sys.argv)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
