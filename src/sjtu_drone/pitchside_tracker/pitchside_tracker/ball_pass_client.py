import rclpy
from rclpy.node import Node
from pitchside_tracker_interfaces.srv import KickBall
from gazebo_msgs.msg import ModelStates
import math


class BallPassClient(Node):
    def __init__(self):
        super().__init__('ball_pass_client')
        self.cli = self.create_client(KickBall, '/kick_ball')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kick_ball service...')

        # Subscribe to Gazebo model states
        self.ball_loaded = False
        self.ball_position = (0.0, 0.0, 0.0)
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.get_logger().info('Waiting for football to appear in Gazebo...')
        while not self.ball_loaded:
            rclpy.spin_once(self)

        self.get_logger().info('Football detected in Gazebo. Ready to pass!')

    def model_states_callback(self, msg: ModelStates):
        if 'football' in msg.name:
            index = msg.name.index('football')
            position = msg.pose[index].position
            self.ball_position = (position.x, position.y, position.z)
            self.ball_loaded = True

    def distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def pass_ball(self, positions, duration=1.0, tolerance=0.05):
        """
        positions: list of (x,y,z)
        duration: seconds for each pass
        tolerance: distance threshold to consider the pass complete
        """
        for i in range(len(positions)-1):
            start = positions[i]
            end = positions[i+1]

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

            if future.result().success:
                self.get_logger().info(f'Pass command sent: {future.result().message}')
            else:
                self.get_logger().warn(f'Pass command failed: {future.result().message}')
                continue  # skip waiting if request failed

            # Wait until ball reaches the target position
            while self.distance(self.ball_position, end) > tolerance:
                rclpy.spin_once(self, timeout_sec=0.05)

            self.get_logger().info(f'Ball reached {end}')


def main():
    rclpy.init()
    client = BallPassClient()

    # Example sequence: ball moves along 4 points
    positions = [
        (0.0, 0.0, 1.1),   # start
        (10.0, 0.0, 1.1),   # first pass
        (10.0, 10.0, 1.1),   # second pass
        (0.0, 5.0, 1.1),    # third pass
        (0.0, 0.0, 1.1)
    ]

    client.pass_ball(positions, duration=20.0)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
