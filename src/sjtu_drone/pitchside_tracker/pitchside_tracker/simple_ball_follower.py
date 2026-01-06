import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time


class SimpleBallFollower(Node):
    def __init__(self):
        super().__init__('simple_ball_follower')

        # Subscribe to detected ball position
        self.sub = self.create_subscription(
            Point,
            'ball_position',
            self.ball_cb,
            10
        )

        # Publish drone velocity
        self.cmd_pub = self.create_publisher(
            Twist,
            '~/cmd_vel',
            10
        )

        # Camera resolution (MUST match your camera)
        self.img_w = 640
        self.img_h = 480

        # Gains (SAFE START)
        self.k_yaw = 0.5
        self.k_z = 0.4

        # Command limits
        self.max_yaw = 0.4
        self.max_z = 0.3

        # Ball timeout
        self.last_seen = time.time()
        self.timeout = 0.5  # seconds

        # Safety timer
        self.create_timer(0.1, self.safety_stop)

        self.get_logger().info("Simple ball follower started")

    def ball_cb(self, msg):
        self.last_seen = time.time()

        cx = msg.x
        cy = msg.y

        # Normalize pixel error
        ex = (cx - self.img_w / 2) / (self.img_w / 2)
        ey = (cy - self.img_h / 2) / (self.img_h / 2)

        cmd = Twist()

        # Rotate to center ball horizontally
        cmd.angular.z = -self.k_yaw * ex

        # Move up/down to center vertically
        cmd.linear.z = -self.k_z * ey

        # Clamp values
        cmd.angular.z = max(min(cmd.angular.z, self.max_yaw), -self.max_yaw)
        cmd.linear.z = max(min(cmd.linear.z, self.max_z), -self.max_z)

        self.cmd_pub.publish(cmd)

    def safety_stop(self):
        if time.time() - self.last_seen > self.timeout:
            self.cmd_pub.publish(Twist())  # hover


def main(args=None):
    rclpy.init(args=args)
    node = SimpleBallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
