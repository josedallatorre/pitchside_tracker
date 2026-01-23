import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time


class SimpleBallFollower(Node):
    def __init__(self):
        super().__init__('simple_ball_follower')

        self.sub = self.create_subscription(Point, 'ball_position', self.ball_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'simple_drone/cmd_vel', 10)

        self.img_w = 640
        self.img_h = 480

        # Gains
        self.k_yaw = 0.5
        self.k_z = 0.4
        self.k_x_base = 0.0005    # base depth gain
        self.k_x_speedup = 0.001  # additional gain if ball is approaching fast

        # Limits
        self.max_yaw = 0.4
        self.max_z = 0.3
        self.max_x = 0.8           # increase max forward speed

        # Desired ball size (controls distance)
        self.target_area = 900

        self.last_seen = time.time()
        self.timeout = 0.5

        # Track previous area to detect ball speed
        self.prev_area = None
        self.prev_time = None

        self.create_timer(0.1, self.safety_stop)

        self.get_logger().info("Ball follower running with adaptive forward speed")

    def ball_cb(self, msg):
        now = time.time()
        self.last_seen = now

        cx, cy, area = msg.x, msg.y, msg.z

        ex = (cx - self.img_w/2) / (self.img_w/2)
        ey = (cy - self.img_h/2) / (self.img_h/2)
        ez = self.target_area - area

        # Estimate approach speed
        approach_speed = 0.0
        if self.prev_area is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                approach_speed = (area - self.prev_area) / dt  # positive if ball is getting bigger (approaching)

        self.prev_area = area
        self.prev_time = now

        # Increase forward speed if ball is approaching
        k_x_dynamic = self.k_x_base + self.k_x_speedup * max(approach_speed, 0.0)

        cmd = Twist()
        cmd.angular.z = -self.k_yaw * ex
        cmd.linear.z = -self.k_z * ey
        cmd.linear.x = k_x_dynamic * ez

        # Apply limits
        cmd.angular.z = max(min(cmd.angular.z, self.max_yaw), -self.max_yaw)
        cmd.linear.z = max(min(cmd.linear.z, self.max_z), -self.max_z)
        cmd.linear.x = max(min(cmd.linear.x, self.max_x), 0.0)  # only forward

        self.cmd_pub.publish(cmd)

    def safety_stop(self):
        if time.time() - self.last_seen > self.timeout:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimpleBallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
