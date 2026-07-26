"""Unit tests for planner_node's cache/join/plan/assemble pipeline. Doesn't
require a live action client, motion_prediction, risk_assessment, or Gazebo
-- builds fabricated Odometry/ObstacleRiskArray/PredictedTrajectoryArray
inputs and drives the staged private methods directly, the same
direct-method-call pattern test_risk_node.py uses for risk_node.

_execute_callback's own blocking action-server loop is intentionally not
unit-tested here -- it needs a spinning executor for create_rate()/sleep() to
function, which is exactly the kind of live-ROS-graph behaviour this
workspace's convention (PROJECT_CONTEXT.md section 12) defers to manual/live
smoke testing instead."""
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from interfaces.msg import ObstacleRisk, ObstacleRiskArray, PredictedTrajectory, PredictedTrajectoryArray, \
    TrajectoryPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from dynamic_planner.planner_node import PlannerNode


def _odom(position=(0.0, 0.0, 0.0), yaw=0.0, linear_velocity=0.0, angular_velocity=0.0):
    odom = Odometry()
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
    odom.pose.pose.orientation.z = np.sin(yaw / 2.0)
    odom.pose.pose.orientation.w = np.cos(yaw / 2.0)
    odom.twist.twist.linear.x = linear_velocity
    odom.twist.twist.angular.z = angular_velocity
    return odom


def _risk(track_id=1, threat_level=ObstacleRisk.THREAT_LOW, time_to_collision=-1.0, risk_score=0.0):
    risk = ObstacleRisk()
    risk.track_id = track_id
    risk.threat_level = threat_level
    risk.time_to_collision = time_to_collision
    risk.risk_score = risk_score
    return risk


def _risks_array(*risks):
    array = ObstacleRiskArray()
    array.risks = list(risks)
    return array


def _point(offset_sec, position=(1.0, 0.0, 0.0)):
    point = TrajectoryPoint()
    point.stamp = TimeMsg(sec=0, nanosec=int(round(offset_sec * 1e9)))
    point.position.x, point.position.y, point.position.z = position
    return point


def _trajectory(track_id=1, points=None):
    trajectory = PredictedTrajectory()
    trajectory.track_id = track_id
    trajectory.points = points if points is not None else [_point(0.1)]
    return trajectory


def _trajectories_array(*trajectories):
    array = PredictedTrajectoryArray()
    array.header = Header()
    array.header.frame_id = 'world'
    array.header.stamp = TimeMsg(sec=0, nanosec=0)
    array.trajectories = list(trajectories)
    return array


def test_control_loop_does_nothing_without_an_active_goal():
    rclpy.init()
    try:
        node = PlannerNode()
        published = []
        node._cmd_pub.publish = published.append

        node._odom_callback(_odom())
        node._control_loop()

        assert published == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_control_loop_does_nothing_without_odom():
    rclpy.init()
    try:
        node = PlannerNode()
        published = []
        node._cmd_pub.publish = published.append

        node._active_goal_position = np.array([5.0, 0.0, 0.0])
        node._control_loop()

        assert published == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_control_loop_stops_and_marks_goal_reached_within_tolerance():
    rclpy.init()
    try:
        node = PlannerNode()
        published = []
        node._cmd_pub.publish = published.append

        node._odom_callback(_odom(position=(4.95, 0.0, 0.0)))
        node._active_goal_position = np.array([5.0, 0.0, 0.0])
        node._control_loop()

        assert node._goal_reached is True
        assert len(published) == 1
        assert published[0].linear.x == 0.0
        assert published[0].angular.z == 0.0
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_control_loop_publishes_a_command_when_goal_is_far_and_clear():
    rclpy.init()
    try:
        node = PlannerNode()
        published = []
        node._cmd_pub.publish = published.append

        node._odom_callback(_odom())
        node._active_goal_position = np.array([5.0, 0.0, 0.0])
        node._control_loop()

        assert len(published) == 1
        assert node._goal_reached is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_control_loop_emergency_stops_on_imminent_critical_threat():
    rclpy.init()
    try:
        node = PlannerNode()
        published = []
        node._cmd_pub.publish = published.append

        node._odom_callback(_odom())
        node._active_goal_position = np.array([5.0, 0.0, 0.0])
        node._risks_callback(_risks_array(
            _risk(track_id=1, threat_level=ObstacleRisk.THREAT_CRITICAL, time_to_collision=0.2)))
        node._control_loop()

        assert len(published) == 1
        assert published[0].linear.x == 0.0
        assert published[0].angular.z == 0.0
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_emergency_stop_not_triggered_by_a_distant_critical_threat():
    rclpy.init()
    try:
        node = PlannerNode()
        node._risks_callback(_risks_array(
            _risk(track_id=1, threat_level=ObstacleRisk.THREAT_CRITICAL, time_to_collision=5.0)))
        assert node._emergency_stop_triggered() is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_emergency_stop_not_triggered_when_ttc_is_negative_one():
    rclpy.init()
    try:
        node = PlannerNode()
        node._risks_callback(_risks_array(
            _risk(track_id=1, threat_level=ObstacleRisk.THREAT_CRITICAL, time_to_collision=-1.0)))
        assert node._emergency_stop_triggered() is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_join_obstacles_skips_tracks_without_a_matching_risk():
    rclpy.init()
    try:
        node = PlannerNode()
        node._trajectories_callback(_trajectories_array(_trajectory(track_id=1)))
        # No corresponding risk cached for track_id=1.
        assert node._join_obstacles() == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_join_obstacles_combines_matching_trajectory_and_risk():
    rclpy.init()
    try:
        node = PlannerNode()
        node._trajectories_callback(_trajectories_array(_trajectory(track_id=7, points=[_point(0.1, (2.0, 0.0, 0.0))])))
        node._risks_callback(_risks_array(_risk(track_id=7)))

        obstacles = node._join_obstacles()
        assert len(obstacles) == 1
        assert obstacles[0].track_id == 7
        assert np.allclose(obstacles[0].positions[0], [2.0, 0.0, 0.0])
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_goal_callback_accepts_when_idle():
    rclpy.init()
    try:
        node = PlannerNode()
        from rclpy.action import GoalResponse
        assert node._goal_callback(None) == GoalResponse.ACCEPT
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_goal_callback_rejects_when_a_goal_is_already_active():
    rclpy.init()
    try:
        node = PlannerNode()
        from rclpy.action import GoalResponse
        node._active_goal_position = np.array([1.0, 1.0, 0.0])
        assert node._goal_callback(None) == GoalResponse.REJECT
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_build_cmd_vel_maps_command_fields():
    rclpy.init()
    try:
        node = PlannerNode()
        from dynamic_planner.local_planner import PlannedCommand
        twist = node._build_cmd_vel(PlannedCommand(linear_velocity=0.3, angular_velocity=-0.4, admissible=True))
        assert twist.linear.x == 0.3
        assert twist.angular.z == -0.4
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_build_global_path_starts_at_robot_and_ends_at_goal():
    rclpy.init()
    try:
        node = PlannerNode()
        node._odom_callback(_odom(position=(0.0, 0.0, 0.0)))
        path = node._build_global_path(np.array([3.0, 0.0, 0.0]))

        assert path.header.frame_id == 'world'
        assert np.isclose(path.poses[0].pose.position.x, 0.0)
        assert np.isclose(path.poses[-1].pose.position.x, 3.0)
        node.destroy_node()
    finally:
        rclpy.shutdown()
