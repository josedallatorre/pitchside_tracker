import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import time


class SimpleBallFollower(Node):
    def __init__(self):
        super().__init__('simple_ball_follower')
        self.yolo_ready = False

        self.ball_sub = self.create_subscription(Point, 'ball_position', self.ball_cb, 10)
        self.ready_sub = self.create_subscription(Bool, '/yolo_tracker/ready', self.cb_ready, 10)
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
        self.max_x = 0.6           # max forward/backward speed

        # Desired ball size (controls distance)
        self.target_area = 900

        self.last_seen = time.time()
        self.timeout = 0.5

        # Track previous area to detect ball speed
        self.prev_area = None
        self.prev_time = None

        self.create_timer(0.1, self.safety_stop)

    def cb_ready(self, msg):
        self.yolo_ready = msg.data
        if msg.data:
            self.get_logger().info("YOLO tracker READY")

    
    def ball_cb(self, msg):
        if not self.yolo_ready:
            return
        now = time.time()
        self.last_seen = now

        cx, cy, area = msg.x, msg.y, msg.z

        # ---- AREA FILTER (ADD HERE) ----
        alpha = 0.7
        if self.prev_area is None:
            filtered_area = area
        else:
            filtered_area = alpha * self.prev_area + (1 - alpha) * area

        area = filtered_area


        ex = (cx - self.img_w/2) / (self.img_w/2)
        ey = (cy - self.img_h/2) / (self.img_h/2)
        ez = self.target_area - area

        # Estimate approach speed
        approach_speed = 0.0
        if self.prev_area is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                approach_speed = (area - self.prev_area) / dt  # positive if ball is approaching

        self.prev_area = area
        self.prev_time = now
        
        # ---- DISTANCE CONTROL (ADD HERE) ----

        distance_deadband = 50

        if abs(ez) < distance_deadband:
            linear_x = 0.0
        else:
            # PD control (stable)
            linear_x = self.k_x_base * ez - 0.0003 * approach_speed

        # Clamp
        linear_x = max(min(linear_x, self.max_x), -self.max_x)

        # Ignore micro-commands
        if abs(linear_x) < 0.02:
            linear_x = 0.0

        linear_x = max(min(linear_x, self.max_x), -self.max_x)  # allow negative => backward

        # Compute yaw and vertical
        cmd = Twist()
        cmd.angular.z = -self.k_yaw * ex
        cmd.linear.z = -self.k_z * ey
        cmd.linear.x = linear_x

        # Apply limits
        cmd.angular.z = max(min(cmd.angular.z, self.max_yaw), -self.max_yaw)
        cmd.linear.z = max(min(cmd.linear.z, self.max_z), -self.max_z)

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
