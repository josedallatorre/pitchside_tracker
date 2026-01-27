import rclpy
from rclpy.node import Node
from pitchside_tracker_interfaces.srv import KickBall
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
import math


class BallPassClient(Node):
    def __init__(self):
        super().__init__('ball_pass_client')

        # ---- Service client ----
        self.cli = self.create_client(KickBall, '/kick_ball')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kick_ball service...')

        # ---- State ----
        self.yolo_loaded = False
        self.ball_loaded = False
        self.ball_position = (0.0, 0.0, 0.0)

        # ---- Subscribers ----
        self.create_subscription(
            Bool,
            '/yolo_tracker/ready',
            self.yolo_ready_callback,
            10
        )

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # ---- Wait for YOLO ----
        self.get_logger().info('Waiting for YOLO to load...')
        while rclpy.ok() and not self.yolo_loaded:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('YOLO is ready. Ready to pass!')

    # ---------------- Callbacks ----------------

    def yolo_ready_callback(self, msg: Bool):
        if msg.data:
            self.yolo_loaded = True

    def model_states_callback(self, msg: ModelStates):
        if 'football' in msg.name:
            idx = msg.name.index('football')
            pos = msg.pose[idx].position
            self.ball_position = (pos.x, pos.y, pos.z)
            self.ball_loaded = True

    # ---------------- Helpers ----------------

    def distance(self, a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    # ---------------- Main Logic ----------------

    def pass_ball(self, positions, duration=1.0, tolerance=0.05):
        """
        positions: list of (x, y, z)
        duration: seconds for each pass
        tolerance: distance threshold to consider pass complete
        """

        # Wait until ball is detected
        self.get_logger().info('Waiting for ball detection...')
        while rclpy.ok() and not self.ball_loaded:
            rclpy.spin_once(self, timeout_sec=0.1)

        for i in range(len(positions) - 1):
            start = positions[i]
            end = positions[i + 1]

            self.get_logger().info(
                f'Passing ball from {start} to {end} over {duration}s'
            )

            req = KickBall.Request()
            req.x = end[0]
            req.y = end[1]
            req.z = end[2]
            req.duration = duration

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if not future.result() or not future.result().success:
                self.get_logger().warn(
                    f'Pass failed: {future.result().message if future.result() else "No response"}'
                )
                continue

            self.get_logger().info('Pass command sent, waiting for ball...')

            # Wait until ball reaches target
            while rclpy.ok() and self.distance(self.ball_position, end) > tolerance:
                rclpy.spin_once(self, timeout_sec=0.05)

            self.get_logger().info(f'Ball reached {end}')


def main():
    rclpy.init()
    client = BallPassClient()

    positions = [
        (0.0, 0.0, 1.1),
        (60.0, 0.0, 1.1),
        (60.0, 40.0, 1.1),
        (0.0, 40.0, 1.1),
        (0.0, 0.0, 1.1),
    ]

    client.pass_ball(positions, duration=200.0)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
