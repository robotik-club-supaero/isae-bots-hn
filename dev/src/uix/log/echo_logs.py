import secrets 
from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

def level_to_string(level):
    if level == 10:
        return "DEBUG"
    elif level == 20:
        return "INFO"
    elif level == 30:
        return "WARN"
    elif level == 40:
        return "ERROR"
    elif level == 50:
        return "FATAL"
    else:
        return level

class LogSubscriber(Node):
    def __init__(self, node_name):
        super().__init__(f'log_subscriber_{secrets.token_hex(16)}')
        self.node_name = node_name
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            10
        )

    def log_callback(self, msg):
        if msg.name == self.node_name:
            print(f"{level_to_string(msg.level)}: {msg.msg}")

def main(args=None):

    p = ArgumentParser()
    p.add_argument("node")
    args = p.parse_args()

    rclpy.init()
    log_subscriber = LogSubscriber(args.node)
    try:
        rclpy.spin(log_subscriber)
    finally:
        log_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
