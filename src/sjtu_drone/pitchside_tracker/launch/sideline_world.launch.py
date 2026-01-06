from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pitchside_tracker',
            executable='image_viewer',
            name='image_viewer',
            parameters=[{
                'image_topic': '/simple_drone/front/image_raw'
            }],
            output='screen'
        )
    ])
