import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Not a LaunchConfiguration: the world rarely changes and keeping it a plain constant
# avoids needing an OpaqueFunction just to fold it into the ros_gz_bridge topic string.
WORLD_NAME = 'multi_obstacle_arena'


def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    world_path = os.path.join(pkg_share, 'worlds', f'{WORLD_NAME}.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'beetlebot.urdf.xacro')
    config_path = os.path.join(pkg_share, 'config', 'simulation_params.yaml')

    with open(config_path) as f:
        sim_params = yaml.safe_load(f)['simulation_manager']['ros__parameters']

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the Gazebo /clock')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items())

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen')

    # Spawned via -topic robot_description: robot_state_publisher latches the parsed
    # URDF onto that topic (std_msgs/String) specifically to support this pattern.
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', WORLD_NAME,
                '-name', 'beetlebot',
                '-topic', 'robot_description',
                '-x', str(sim_params['robot_spawn_x']),
                '-y', str(sim_params['robot_spawn_y']),
                '-z', '0.05',
                '-Y', str(sim_params['robot_spawn_yaw']),
            ],
            output='screen')])

    # Ground truth for Module 3: each obstacle publishes its own named pose via a
    # PosePublisher plugin (see simulation_manager.OBSTACLE_POSE_PUBLISHER_PLUGIN and
    # *_pose_topic()) on its own per-model topic -- confirmed on a real Harmonic
    # install that this build's PosePublisher ignores a custom <topic> override, so a
    # single shared bridge line isn't possible; one line per obstacle is generated here
    # from the same num_static_obstacles/num_dynamic_obstacles config simulation_manager
    # spawns from. Filtered downstream by name prefix ("static_obstacle_" /
    # "dynamic_obstacle_"). /world/<world>/pose/info is NOT used for this:
    # SceneBroadcaster's Pose_V never sets the header data the tf2_msgs bridge
    # conversion reads, so child_frame_id would come through empty.
    obstacle_pose_bridge_args = [
        f'/model/static_obstacle_{i}/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        for i in range(sim_params['num_static_obstacles'])
    ] + [
        f'/model/dynamic_obstacle_{i}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        for i in range(sim_params['num_dynamic_obstacles'])
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/pi_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ] + obstacle_pose_bridge_args,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Obstacle spawning also needs the world's -create service up, so it waits behind
    # the robot rather than racing it.
    simulation_manager = TimerAction(
        period=8.0,
        actions=[Node(
            package='simulation',
            executable='simulation_manager',
            parameters=[config_path, {'use_sim_time': use_sim_time}],
            output='screen')])

    return LaunchDescription([
        declare_use_sim_time,
        gz_sim,
        robot_state_publisher,
        bridge,
        spawn_robot,
        simulation_manager,
    ])
