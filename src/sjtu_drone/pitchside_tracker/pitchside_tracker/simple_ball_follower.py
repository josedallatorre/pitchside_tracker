import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import time


class SimpleBallFollower(Node):
    def __init__(self):
        super().__init__('dynamic_ball_follower')

        # State
        self.yolo_ready = False
        self.last_seen = time.time()
        self.prev_area = None
        self.prev_time = None
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.prev_linear_z = 0.0

        # Subscribers
        self.ball_sub = self.create_subscription(Point, 'ball_position', self.ball_cb, 10)
        self.ready_sub = self.create_subscription(Bool, '/yolo_tracker/ready', self.cb_ready, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'simple_drone/cmd_vel', 10)

        # Image size
        self.img_w = 640
        self.img_h = 480

        # Gains
        self.k_yaw = 0.8          # Horizontal control
        self.k_z = 0.6            # Vertical control
        self.k_x_base = 0.0008    # Forward/backward base gain
        self.k_x_speedup = 0.0015 # Forward/backward additional gain for fast approach
        self.k_yaw_speedup = 0.4  # Angular boost for fast lateral movement

        # Max speeds
        self.max_yaw = 1.0
        self.max_z = 0.6
        self.max_x = 1.0

        # Desired area (distance from ball)
        self.target_area = 1300
        self.distance_deadband = 30

        # Timeout for safety stop
        self.timeout = 0.5
        self.create_timer(0.1, self.safety_stop)

        self.get_logger().info("DynamicBallFollower node started")

    def cb_ready(self, msg: Bool):
        self.yolo_ready = msg.data
        if msg.data:
            self.get_logger().info("YOLO tracker READY")

    def ball_cb(self, msg: Point):
        if not self.yolo_ready:
            return

        now = time.time()
        self.last_seen = now

        # Ball info
        cx, cy, area = msg.x, msg.y, msg.z

        # ---- Smooth area to reduce jitter ----
        alpha = 0.7
        if self.prev_area is None:
            filtered_area = area
        else:
            filtered_area = alpha * self.prev_area + (1 - alpha) * area
        area = filtered_area

        # Compute errors
        ex = (cx - self.img_w / 2) / (self.img_w / 2)  # Horizontal (-1 to 1)
        ey = (cy - self.img_h / 2) / (self.img_h / 2)  # Vertical (-1 to 1)
        ez = self.target_area - area                     # Forward/backward

        # ---- Estimate approach speed (forward/backward) ----
        approach_speed = 0.0
        lateral_speed = 0.0
        if self.prev_area is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                approach_speed = (area - self.prev_area) / dt           # positive if ball is approaching
                lateral_speed = (cx - self.prev_cx) / dt               # pixels/sec lateral

        self.prev_area = area
        self.prev_cx = cx
        self.prev_time = now

        # ---- Forward/backward control (linear.x) ----
        if abs(ez) < self.distance_deadband:
            linear_x = 0.0
        else:
            # Proportional + dynamic boost for fast approach
            linear_x = self.k_x_base * ez + self.k_x_speedup * approach_speed

        # ---- Horizontal control (angular.z) ----
        angular_z = -self.k_yaw * ex - self.k_yaw_speedup * lateral_speed / self.img_w

        # ---- Vertical control (linear.z) ----
        linear_z = -self.k_z * ey

        # ---- Apply low-pass smoothing ----
        alpha_filter = 0.6
        linear_x = alpha_filter * self.prev_linear_x + (1 - alpha_filter) * linear_x
        self.prev_linear_x = linear_x

        angular_z = alpha_filter * self.prev_angular_z + (1 - alpha_filter) * angular_z
        self.prev_angular_z = angular_z

        linear_z = alpha_filter * self.prev_linear_z + (1 - alpha_filter) * linear_z
        self.prev_linear_z = linear_z

        # ---- Clamp to max speeds ----
        linear_x = max(min(linear_x, self.max_x), -self.max_x)
        angular_z = max(min(angular_z, self.max_yaw), -self.max_yaw)
        linear_z = max(min(linear_z, self.max_z), -self.max_z)

        # ---- Publish command ----
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.z = linear_z
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

    def safety_stop(self):
        # Stop drone if no ball seen recently
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
