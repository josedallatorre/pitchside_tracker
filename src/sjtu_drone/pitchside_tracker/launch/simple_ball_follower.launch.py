from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pitchside_tracker',
            executable='simple_ball_follower',
            name='simple_ball_follower',
        ),
    ])
