import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cognitive_perception')
    config_path = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the Gazebo /clock')

    perception_node = Node(
        package='cognitive_perception',
        executable='perception_node',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen')

    # Phase-2 placeholders: wired to their real sensor topics today so the launch
    # entry and topic names are already correct, but produce no detections yet --
    # perception_node does not depend on either of these in Phase 1 (see README).
    camera_node = Node(
        package='cognitive_perception',
        executable='camera_node',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen')

    lidar_node = Node(
        package='cognitive_perception',
        executable='lidar_node',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        perception_node,
        camera_node,
        lidar_node,
    ])
