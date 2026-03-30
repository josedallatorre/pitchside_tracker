import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import time


class BallFollower(Node):
    def __init__(self):
        super().__init__('dynamic_ball_follower')

        # State 
        self.yolo_ready = False
        self.last_seen  = time.time()
        
        # area, error for x and y filtered
        self.area_f     = None
        self.ex_f       = 0.0
        self.ey_f       = 0.0

        # velocity for area, x and y filtered
        self.area_vel   = 0.0
        self.ex_vel     = 0.0
        self.ey_vel     = 0.0

        self.prev_time  = None

        # ROS I/O 
        self.ball_sub  = self.create_subscription(Point, 'ball_position',       self.ball_cb,  10)
        self.ready_sub = self.create_subscription(Bool,  '/yolo_tracker/ready', self.cb_ready, 10)
        self.cmd_pub   = self.create_publisher(Twist, 'simple_drone/cmd_vel', 10)

        # Image geometry 
        self.img_w = 1920
        self.img_h = 1080

        # Signal filter alphas 
        # Higher = more responsive, lower = smoother
        # fast ball needs faster signal tracking
        self.alpha_area = 0.60
        self.alpha_ex   = 0.50
        self.alpha_ey   = 0.50
        # Signal Filtering EMA
        # EMA = Exponential Moving Average
        # Velocity EMA: moderate, prevents noisy derivative
        self.alpha_vel  = 0.35

        # Distance control
        self.target_area = 8000

        # PD gains: forward/back 
        # PD = proportional derivative
        self.kp_x       =  0.010
        # D term increased 6x: must be large enough to actually brake
        # at max approach velocity (~1000 px²/s → braking cmd = 0.0018*1000 = 1.8 → clamped to 0.45)
        self.kd_x_brake =  0.0018  # always subtracted when area_vel > 0

        # PD gains: yaw
        self.kp_yaw =  1.0
        self.kd_yaw =  0.4   # note: applied as subtraction below

        # PD gains: altitude 
        self.kp_z   =  0.5
        self.kd_z   =  0.2   # note: applied as subtraction below

        # Vertical tracking offset 
        self.vert_offset = 0.20

        # Output limits
        self.max_x   = 0.15
        self.max_yaw = 1.2
        self.max_z   = 0.5

        # Centering scale 
        self.center_min_scale = 0.40

        # Timeout
        self.timeout = 2.0

        self.create_timer(0.05, self.safety_stop)
        self.get_logger().info("BallFollower started")

    # ──────────────────────────────────────────────────────────────────────────
    # callback, signal that yolo model is ready for detection
    def cb_ready(self, msg: Bool):
        self.yolo_ready = msg.data

    # ──────────────────────────────────────────────────────────────────────────
    # callback, return information of the ball whenever the model detect one
    def ball_cb(self, msg: Point):
        if not self.yolo_ready:
            return

        now = time.time()
        dt  = (now - self.prev_time) if self.prev_time is not None else 0.033 # delta time
        dt  = max(min(dt, 0.20), 0.005)
        self.prev_time = now
        self.last_seen = now

        cx, cy, area = msg.x, msg.y, msg.z

        ex_raw = (cx - self.img_w / 2) / (self.img_w / 2)
        ey_raw = (cy - self.img_h / 2) / (self.img_h / 2)

        # Bootstrap
        if self.area_f is None:
            self.area_f = area
            self.ex_f   = ex_raw
            self.ey_f   = ey_raw
            return

        # Filter signals (before differentiation)
        area_f = (1 - self.alpha_area) * self.area_f + self.alpha_area * area
        ex_f   = (1 - self.alpha_ex)   * self.ex_f   + self.alpha_ex   * ex_raw
        ey_f   = (1 - self.alpha_ey)   * self.ey_f   + self.alpha_ey   * ey_raw

        # Velocity from filtered signals 
        raw_area_vel = (area_f - self.area_f) / dt
        raw_ex_vel   = (ex_f   - self.ex_f)   / dt
        raw_ey_vel   = (ey_f   - self.ey_f)   / dt

        self.area_vel = (1 - self.alpha_vel) * self.area_vel + self.alpha_vel * raw_area_vel
        self.ex_vel   = (1 - self.alpha_vel) * self.ex_vel   + self.alpha_vel * raw_ex_vel
        self.ey_vel   = (1 - self.alpha_vel) * self.ey_vel   + self.alpha_vel * raw_ey_vel

        self.area_f = area_f
        self.ex_f   = ex_f
        self.ey_f   = ey_f

        # Centering scale
        off_center   = abs(ex_f) + abs(ey_f)
        center_scale = max(self.center_min_scale, 1.0 - 0.3 * off_center)

        # Forward/back PD
        ez = self.target_area - area_f

        # D term active even inside deadband, critical for braking overshoot
        # Braking: if area_vel > 0 (ball approaching), push backward
        # Chasing: if area_vel < 0 (ball receding), push forward
        d_term = -self.kd_x_brake * self.area_vel

        if abs(ez) < 400:
            # Inside deadband: only brake, don't drive
            cmd_x = d_term * center_scale
        else:
            cmd_x = (self.kp_x * ez + d_term) * center_scale

        # Yaw PD
        cmd_yaw = -(self.kp_yaw * ex_f + self.kd_yaw * self.ex_vel)

        # Altitude PD
        ey_err  = ey_f - self.vert_offset
        cmd_z   = -(self.kp_z * ey_err + self.kd_z * self.ey_vel)
        
        # Notes:
        # No output smoothing, signal side EMA is sufficient
        # Output EMA was delaying the D term by 3+ frames, defeating its purpose

        # Dynamic Clamp
        # Far away = allow faster chase, close = slow down
        distance_scale = min(1.0, abs(ez) / 3000.0)  # ramps from 0→1 over 3000px² error
        effective_max_x = self.max_x * distance_scale + 0.05  # floor of 0.05 m/s
        cmd_x = max(min(cmd_x, effective_max_x), -effective_max_x)
        cmd_yaw = max(min(cmd_yaw, self.max_yaw), -self.max_yaw)
        cmd_z   = max(min(cmd_z,   self.max_z),   -self.max_z)
        
        # Create and publish the twist
        cmd = Twist()
        cmd.linear.x  = cmd_x
        cmd.linear.z  = cmd_z
        cmd.angular.z = cmd_yaw
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"area={area_f:.0f}  ez={ez:.0f}  area_vel={self.area_vel:.1f}"
            f"  P={self.kp_x*ez:.3f}  D={d_term:.3f}"
            f"  cmd_x={cmd_x:.3f}  yaw={cmd_yaw:.3f}  z={cmd_z:.3f}"
        )

    # safety stop based on timeout
    # Drone stop chasing the ball if the model didn't detect any ball in the last <timeout> second 
    def safety_stop(self):
        if time.time() - self.last_seen > self.timeout:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()