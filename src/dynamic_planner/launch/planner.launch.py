import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_planner')
    config_path = os.path.join(pkg_share, 'config', 'planner_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the Gazebo /clock')

    planner_node = Node(
        package='dynamic_planner',
        executable='planner_node',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        planner_node,
    ])
