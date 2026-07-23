from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_orchestrator',
            executable='task_orchestrator_node',
            name='task_orchestrator',
            output='screen',
        )
    ])
