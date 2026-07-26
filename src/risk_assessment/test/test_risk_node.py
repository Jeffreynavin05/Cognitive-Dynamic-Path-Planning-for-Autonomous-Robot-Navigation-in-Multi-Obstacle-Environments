"""Unit tests for risk_node's cache/compute/assemble pipeline. Doesn't require
a live subscriber/publisher, motion_prediction, or Gazebo -- builds fabricated
Odometry and PredictedTrajectoryArray inputs and drives _odom_callback/
_trajectories_callback directly, the same direct-method-call pattern
test_prediction_node.py uses for prediction_node."""
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from interfaces.msg import PredictedTrajectory, PredictedTrajectoryArray, TrajectoryPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from risk_assessment.risk_node import RiskNode


def _odom(position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)):
    odom = Odometry()
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
    odom.pose.pose.orientation.w = 1.0  # identity: body frame == world frame
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocity
    return odom


def _point(offset_sec, position=(1.0, 0.0, 0.0), velocity=(-1.0, 0.0, 0.0), variance=0.01):
    point = TrajectoryPoint()
    point.stamp = TimeMsg(sec=0, nanosec=int(round(offset_sec * 1e9)))
    point.position.x, point.position.y, point.position.z = position
    point.velocity.x, point.velocity.y, point.velocity.z = velocity
    point.covariance = (np.eye(3) * variance).flatten().tolist()
    return point


def _collision_course_trajectory(track_id=0, num_steps=5, step_sec=0.1, start_x=1.0, speed=2.0):
    points = [
        _point(step_sec * i, position=(start_x - speed * step_sec * i, 0.0, 0.0), velocity=(-speed, 0.0, 0.0))
        for i in range(1, num_steps + 1)
    ]
    trajectory = PredictedTrajectory()
    trajectory.track_id = track_id
    trajectory.model_name = 'constant_velocity'
    trajectory.points = points
    return trajectory


def _trajectories_array(*trajectories):
    array = PredictedTrajectoryArray()
    array.header = Header()
    array.header.frame_id = 'world'
    array.header.stamp = TimeMsg(sec=0, nanosec=0)
    array.trajectories = list(trajectories)
    return array


def test_nothing_is_published_before_odom_is_received():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._trajectories_callback(_trajectories_array(_collision_course_trajectory()))

        assert published == []

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_risk_is_published_once_odom_has_been_received():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._odom_callback(_odom())
        node._trajectories_callback(_trajectories_array(_collision_course_trajectory()))

        assert len(published) == 1
        assert len(published[0].risks) == 1

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_risk_carries_the_source_track_id():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._odom_callback(_odom())
        node._trajectories_callback(_trajectories_array(_collision_course_trajectory(track_id=7)))

        assert published[0].risks[0].track_id == 7

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_multiple_trajectories_each_get_their_own_risk():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._odom_callback(_odom())
        node._trajectories_callback(_trajectories_array(
            _collision_course_trajectory(track_id=1), _collision_course_trajectory(track_id=2)))

        assert len(published[0].risks) == 2
        assert {risk.track_id for risk in published[0].risks} == {1, 2}

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_risk_fields_are_within_their_documented_ranges():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._odom_callback(_odom())
        node._trajectories_callback(_trajectories_array(_collision_course_trajectory()))

        risk = published[0].risks[0]
        assert 0.0 <= risk.risk_score <= 1.0
        assert 0.0 <= risk.path_intersection_prob <= 1.0
        assert risk.threat_level in (0, 1, 2, 3)
        assert risk.time_to_collision == -1.0 or risk.time_to_collision >= 0.0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_header_is_propagated_from_the_trajectories_message():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._odom_callback(_odom())
        node._trajectories_callback(_trajectories_array(_collision_course_trajectory()))

        assert published[0].header.frame_id == 'world'

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_is_called_synchronously_once_per_trajectories_array():
    rclpy.init()
    try:
        node = RiskNode()
        published = []
        node._risks_pub.publish = published.append

        node._odom_callback(_odom())
        node._trajectories_callback(_trajectories_array(_collision_course_trajectory()))
        node._trajectories_callback(_trajectories_array(_collision_course_trajectory()))

        assert len(published) == 2

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_robot_velocity_is_rotated_from_body_frame_into_world_frame():
    rclpy.init()
    try:
        node = RiskNode()

        # 90-degree yaw: body-frame +x becomes world-frame +y.
        odom = _odom(velocity=(1.0, 0.0, 0.0))
        odom.pose.pose.orientation.z = np.sin(np.pi / 4)
        odom.pose.pose.orientation.w = np.cos(np.pi / 4)
        node._odom_callback(odom)

        assert np.allclose(node._robot_velocity, [0.0, 1.0, 0.0], atol=1e-6)

        node.destroy_node()
    finally:
        rclpy.shutdown()
