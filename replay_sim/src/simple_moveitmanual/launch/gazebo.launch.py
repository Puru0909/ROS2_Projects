import os
import xacro
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "simple_moveitmanual"
    package_share = get_package_share_directory(package_name)

    urdf_path = os.path.join(package_share, "urdf", "simplearm.urdf")
    robot_description_raw = xacro.process_file(urdf_path).toxml()

    moveit_config = (
        MoveItConfigsBuilder("simplearm", package_name=package_name)
        .robot_description(file_path=os.path.join(package_share, "urdf", "simplearm.urdf"))
        .robot_description_semantic(file_path="config/simplearm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        package_share, "config", "ros2_controllers.yaml"
    )

    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(package_share, "worlds", "arm_world.world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_file, "verbose": "false"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_raw},
            {"use_sim_time": True},
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "simplearm",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    t_shape_urdf = os.path.join(package_share, "urdf", "t_shape.urdf")
    spawn_t_shape = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", t_shape_urdf,
            "-entity", "t_shape",
            "-x", "0.20",
            "-y", "0.0",
            "-z", "0.0",
            "-Y", "0.0",
        ],
        output="screen",
    )

    highlight_sdf = os.path.join(package_share, "urdf", "highlight_zone.sdf")
    spawn_highlight = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", highlight_sdf,
            "-entity", "highlight_zone",
            "-x", "0.20",   # 0.20 and -0.10
            "-y", "0.32",
            "-z", "0.0",
            "-Y", "0.0",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    delayed_moveit = TimerAction(
        period=8.0,
        actions=[move_group_node],
    )

    rviz_config = os.path.join(package_share, "rviz", "arm.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_t_shape,
        spawn_highlight,
        delayed_controllers,
        delayed_moveit,
        delayed_rviz,
    ])