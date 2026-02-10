from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pitchside_tracker',
            executable='yolo_ball_tracker',
            name='yolo_ball_tracker',
            parameters=[{
                'image_topic': '/simple_drone/front/image_raw',
                'model_path': 'yolov8n.pt',
                'confidence': 0.4
            }],
            output='screen'
        )
    ])
