import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import ModelStates

from pitchside_tracker_interfaces.srv import KickBall


class KickBallServer(Node):

    def __init__(self):
        super().__init__('kick_ball_server')

        # ROS service for kicking the ball
        self.srv = self.create_service(KickBall, 'kick_ball', self.kick_callback)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Subscribe to Gazebo model states to read current ball position
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.ball_position = (0.0, 0.0, 0.0)  # will be updated from Gazebo

        self.timer = None
        self.steps = 0
        self.step = 0

        # camera FPS (controls update frequency)
        self.fps = 20.0

    def model_states_callback(self, msg: ModelStates):
        try:
            index = msg.name.index('football')
            position = msg.pose[index].position
            self.ball_position = (position.x, position.y, position.z)
        except ValueError:
            # football not found yet
            pass

    def kick_callback(self, request, response):
        # Use current ball position as start
        self.start = self.ball_position
        self.end = (request.x, request.y, request.z)

        self.steps = int(request.duration * self.fps)
        self.step = 0

        self.timer = self.create_timer(1.0 / self.fps, self.update_ball)

        response.success = True
        response.message = "Kick started"
        return response

    def update_ball(self):
        if self.step >= self.steps:
            self.timer.cancel()
            return

        t = self.step / self.steps

        # linear interpolation for x and y
        x = self.start[0] + t * (self.end[0] - self.start[0])
        y = self.start[1] + t * (self.end[1] - self.start[1])
        # parabolic arc for z
        z = (1 - t) * self.start[2] + t * self.end[2] + 0.5 * 9.81 * t * (1 - t)

        # set new state
        state = EntityState()
        state.name = 'football'
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0

        # call Gazebo service
        req = SetEntityState.Request()
        req.state = state
        if self.client.wait_for_service(timeout_sec=1.0):
            self.client.call_async(req)

        self.step += 1


def main():
    rclpy.init()
    node = KickBallServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
