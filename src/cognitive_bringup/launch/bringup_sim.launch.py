import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# One-command Phase-1 entry point: Gazebo + the full pipeline + visualization,
# all with use_sim_time:=true. See bringup_hardware.launch.py for the Phase-2
# entry point -- the two never share a file, only the pipeline.launch.py they
# both include, per PROJECT_CONTEXT.md's approved "no runtime sim/hardware
# branch" decision for this module.


def generate_launch_description():
    sim_config_path = os.path.join(
        get_package_share_directory('simulation'), 'config', 'simulation_params.yaml')
    with open(sim_config_path) as f:
        sim_params = yaml.safe_load(f)['simulation_manager']['ros__parameters']

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the Gazebo /clock')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2 alongside the pipeline')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('simulation'), 'launch', 'world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # No pipeline message carries a real "world" TF frame (every
    # DetectedObjectArray/TrackedObjectArray/PredictedTrajectoryArray/
    # ObstacleRiskArray header.frame_id is the constant "world" --
    # PROJECT_CONTEXT.md section 8 -- while simulation/launch/world.launch.py's
    # diff-drive plugin only ever publishes odom -> base_footprint on /tf).
    # Without this, RViz has two disconnected TF trees and cannot render the
    # robot model and any pipeline marker/path in the same view. This
    # publishes the one missing link, world -> odom, as a fixed offset equal
    # to the robot's spawn pose (read directly from simulation's own config,
    # not duplicated by hand) -- exact at t=0 by construction, and only ever
    # as accurate afterward as the diff-drive plugin's own odometry
    # integration, the same category of approximation already documented for
    # risk_node's self-projected path (section 9b/14/17). Visualization-only:
    # no pipeline node subscribes to or depends on this transform, and unlike
    # spawn_robot/simulation_manager in simulation/launch/world.launch.py it
    # needs nothing from Gazebo to be up first, so it starts immediately
    # rather than behind a TimerAction.
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', str(sim_params['robot_spawn_x']),
            '--y', str(sim_params['robot_spawn_y']),
            '--z', '0',
            '--yaw', str(sim_params['robot_spawn_yaw']),
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cognitive_bringup'), 'launch', 'pipeline.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'output_topic': '/cmd_vel',
            'use_rviz': use_rviz,
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        world,
        world_to_odom,
        pipeline,
    ])
