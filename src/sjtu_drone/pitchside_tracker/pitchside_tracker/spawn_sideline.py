import rclpy
from rclpy.node import Node

class SidelineSpawner(Node):
    def __init__(self):
        super().__init__('sideline_spawner')
        self.get_logger().info('Sideline drone spawner running!')

def main(args=None):
    rclpy.init(args=args)
    node = SidelineSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
