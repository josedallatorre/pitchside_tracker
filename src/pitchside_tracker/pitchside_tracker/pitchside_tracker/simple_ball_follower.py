import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import time


class SimpleBallFollower(Node):
    def __init__(self):
        super().__init__('dynamic_ball_follower')

        self.yolo_ready = False
        self.last_seen = time.time()

        self.prev_area = None
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.prev_linear_z = 0.0

        self.ball_sub = self.create_subscription(
            Point, 'ball_position', self.ball_cb, 10
        )
        self.ready_sub = self.create_subscription(
            Bool, '/yolo_tracker/ready', self.cb_ready, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, 'simple_drone/cmd_vel', 10
        )

        self.img_w = 640
        self.img_h = 480

        # Gains
        self.k_yaw = 1.6      # FASTER yaw
        self.k_z = 0.8
        self.k_x = 0.0012    # FASTER approach

        # Limits
        self.max_yaw = 2.0
        self.max_z = 0.8
        self.max_x = 0.6

        # Distance control
        self.target_area = 2000
        self.base_deadband = 20

        # Centering thresholds
        self.center_x = 0.25   # relaxed
        self.center_y = 0.25

        self.timeout = 2.0
        self.create_timer(0.1, self.safety_stop)

        self.get_logger().info("Fast Stable Ball Follower started")

    def cb_ready(self, msg: Bool):
        self.yolo_ready = msg.data

    def ball_cb(self, msg: Point):
        if not self.yolo_ready:
            return

        self.last_seen = time.time()

        cx, cy, area = msg.x, msg.y, msg.z

        # --- Area smoothing ---
        alpha_area = 0.6
        area_f = area if self.prev_area is None else (
            alpha_area * self.prev_area + (1 - alpha_area) * area
        )
        self.prev_area = area_f

        # --- Errors ---
        ex = (cx - self.img_w / 2) / (self.img_w / 2)
        ey = (cy - self.img_h / 2) / (self.img_h / 2)
        ez = self.target_area - area_f

        # --- Yaw (aggressive when far) ---
        angular_z = -self.k_yaw * ex * (1 + abs(ex))

        # --- Vertical ---
        linear_z = -self.k_z * ey

        # --- Forward speed ramps with centering ---
        centering_score = max(0.0, 1.0 - (abs(ex) + abs(ey)))
        dynamic_deadband = self.base_deadband + 150 * (1 - centering_score)

        if abs(ez) < dynamic_deadband:
            linear_x = 0.0
        else:
            linear_x = self.k_x * ez * centering_score

        # --- Minimum useful speeds ---
        if 0 < abs(linear_x) < 0.15:
            linear_x = 0.15 * (1 if linear_x > 0 else -1)

        # --- Filtering ---
        alpha = 0.5
        linear_x = alpha * self.prev_linear_x + (1 - alpha) * linear_x
        angular_z = alpha * self.prev_angular_z + (1 - alpha) * angular_z
        linear_z = alpha * self.prev_linear_z + (1 - alpha) * linear_z

        self.prev_linear_x = linear_x
        self.prev_angular_z = angular_z
        self.prev_linear_z = linear_z

        # --- Clamp ---
        linear_x = max(min(linear_x, self.max_x), -self.max_x)
        angular_z = max(min(angular_z, self.max_yaw), -self.max_yaw)
        linear_z = max(min(linear_z, self.max_z), -self.max_z)

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.z = linear_z
        cmd.angular.z = angular_z
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
