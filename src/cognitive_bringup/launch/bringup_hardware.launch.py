import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Phase-2 entry point -- deliberately kept as a separate file from
# bringup_sim.launch.py rather than a sim:=true/false branch inside one file,
# so the two environments can never be cross-wired by a wrong argument at
# launch time (PROJECT_CONTEXT.md's approved design decision for this
# module). Both files include the same pipeline.launch.py; only what
# surrounds it differs.
#
# STUB: this file does not launch any BeetleBot sensor driver (RPLidar C1,
# Pi Camera V1.3, LSM6DSRTR IMU) or the wheel-odometry/motor-controller nodes
# that publish /scan, /imu/data, /pi_camera/image_raw, /wheel/odom, and
# consume /cmd_vel_gate on real hardware -- per PROJECT_CONTEXT.md section 1,
# that BeetleBot-platform bringup is existing infrastructure this project
# assumes exists but does not build, the same assumption already documented
# for /cmd_vel_gate's arbitration node itself (section 9d/17). This file's
# only job is to prove the pipeline-side launch argument seam (use_sim_time,
# output_topic) is already correct and requires no code change for Phase 2 --
# not to be a complete real-robot bringup on its own.


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2 alongside the pipeline')

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cognitive_bringup'), 'launch', 'pipeline.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'output_topic': '/cmd_vel_gate',
            'use_rviz': use_rviz,
        }.items())

    return LaunchDescription([
        declare_use_rviz,
        pipeline,
    ])
