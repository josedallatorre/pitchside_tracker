#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pitchside_tracker_interfaces.srv import KickBall
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
from pitchside_tracker.utils.yaml_loader import load_yaml
from pitchside_tracker.utils.pass_builder import build_pass_positions
from pitchside_tracker.utils.geometry_utils import distance


class BallPassClient(Node):
    def __init__(self):
        super().__init__('ball_pass_client')

        # --- Load config from YAML ---
        self.config = load_yaml() or {}
        player_positions = [tuple(p["position"]) for p in self.config.get("players", [])]
        if not player_positions:
            self.get_logger().warn("No player positions found in YAML. Using default positions.")
            player_positions = [(0.0, 0.0, 1.1), (60.0, 0.0, 1.1)]  # fallback

        # Build the pass positions sequence
        self.pass_positions = build_pass_positions(player_positions)

        # --- Service client ---
        self.cli = self.create_client(KickBall, '/kick_ball')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kick_ball service...')

        # --- State ---
        self.yolo_loaded = False
        self.ball_loaded = False
        self.ball_position = (0.0, 0.0, 0.0)

        # --- Subscribers ---
        self.create_subscription(Bool, '/yolo_tracker/ready', self.yolo_ready_callback, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        # --- Wait for YOLO ---
        self.get_logger().info('Waiting for YOLO tracker to be ready...')
        while rclpy.ok() and not self.yolo_loaded:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('YOLO tracker is ready. Ready to pass!')

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

    # ---------------- Main Logic ----------------
    def pass_ball(self, positions=None, speed=0.5, tolerance=0.5):
        """
        positions: list of (x, y, z)
        speed: ball speed (units per second)
        tolerance: distance threshold to consider pass complete
        """
        if positions is None:
            positions = self.pass_positions

        # Wait until ball is detected
        self.get_logger().info('Waiting for ball detection...')
        while rclpy.ok() and not self.ball_loaded:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Loop through pass positions
        for i in range(len(positions) - 1):
            start = positions[i]
            end = positions[i + 1]

            # Compute distance and duration
            dist = distance(start, end)
            duration = dist / speed

            self.get_logger().info(
                f'Passing ball from {start} to {end} | Distance: {dist:.2f} | Duration: {duration:.2f}s'
            )

            # Send kick request
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

            self.get_logger().info('Pass command sent, waiting for ball to reach target...')

            # Wait until ball reaches the target
            while rclpy.ok() and distance(self.ball_position, end) > tolerance:
                rclpy.spin_once(self, timeout_sec=0.05)

            self.get_logger().info(f'Ball reached {end}')


def main():
    rclpy.init()
    client = BallPassClient()

    # Start passing the ball using the sequence from YAML
    client.pass_ball(speed=0.3)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()