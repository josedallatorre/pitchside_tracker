import rclpy
from rclpy.node import Node

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.get_logger().info('Pitchside Tracker Node has started!')

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
