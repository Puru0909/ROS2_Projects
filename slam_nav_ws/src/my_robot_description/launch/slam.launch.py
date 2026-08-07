import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_share = get_package_share_directory('my_robot_description')
    params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': params_file,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([slam])
