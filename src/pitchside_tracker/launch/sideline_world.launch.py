from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pitchside_tracker')
    world_path = os.path.join(pkg_share, 'worlds', 'football_pitch.world')
    models_path = os.path.join(pkg_share, 'models')

    set_model_path = SetEnvironmentVariable(name='GZ_MODEL_PATH', value=models_path)

    ros_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz'), 'launch', 'gz.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items()
    )

    return LaunchDescription([
        set_model_path,
        ros_gz_launch,
    ])
