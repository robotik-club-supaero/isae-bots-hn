import sys
import rclpy

def main():
    rclpy.init(args=sys.argv)

    # TODO: simulate base roulante
    node = rclpy.create_node("BRN")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()