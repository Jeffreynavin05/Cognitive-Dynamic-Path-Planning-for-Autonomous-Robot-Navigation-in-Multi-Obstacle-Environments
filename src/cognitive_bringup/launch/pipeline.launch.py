import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Environment-agnostic: every node included here is written to be unaware of
# whether it's talking to Gazebo or the real BeetleBot (PROJECT_CONTEXT.md
# section 1's digital-twin thesis). bringup_sim.launch.py and
# bringup_hardware.launch.py both include this file and differ only in what
# they include *around* it (the Gazebo world vs. nothing yet) and which
# use_sim_time/output_topic values they pass in.


def _pkg_launch(package: str, launch_file: str) -> str:
    return os.path.join(get_package_share_directory(package), 'launch', launch_file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    output_topic = LaunchConfiguration('output_topic')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the Gazebo /clock (false on real hardware)')
    declare_output_topic = DeclareLaunchArgument(
        'output_topic', default_value='/cmd_vel',
        description="robot_controller's relay destination: /cmd_vel (sim) or /cmd_vel_gate (hardware)")
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2 with this package\'s bringup.rviz config')

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('cognitive_perception', 'perception.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('cognitive_tracking', 'tracking.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    prediction = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('motion_prediction', 'prediction.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    risk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('risk_assessment', 'risk.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('dynamic_planner', 'planner.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('robot_controller', 'controller.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'output_topic': output_topic}.items())

    visualization_node = Node(
        package='cognitive_bringup',
        executable='visualization_node',
        parameters=[
            os.path.join(get_package_share_directory('cognitive_bringup'), 'config', 'visualization_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('cognitive_bringup'), 'rviz', 'bringup.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        declare_output_topic,
        declare_use_rviz,
        perception,
        tracking,
        prediction,
        risk,
        planner,
        controller,
        visualization_node,
        rviz,
    ])
