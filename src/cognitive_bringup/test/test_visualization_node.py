"""Unit tests for visualization_node's per-topic marker building. Doesn't
require a live subscriber/publisher or Gazebo -- builds fabricated
TrackedObjectArray/PredictedTrajectoryArray/ObstacleRiskArray inputs and
drives _tracks_callback/_trajectories_callback/_risks_callback directly, the
same direct-method-call pattern every other node's tests in this workspace
use."""
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from interfaces.msg import (ObstacleRisk, ObstacleRiskArray, PredictedTrajectory, PredictedTrajectoryArray,
                             TrackedObject, TrackedObjectArray, TrajectoryPoint)
from std_msgs.msg import Header

from cognitive_bringup.visualization_node import VisualizationNode


def _header():
    header = Header()
    header.frame_id = 'world'
    header.stamp = TimeMsg(sec=0, nanosec=0)
    return header


def _track(track_id=0, status=TrackedObject.STATUS_CONFIRMED, position=(1.0, 2.0, 0.0), size=(0.4, 0.4, 0.6)):
    track = TrackedObject()
    track.track_id = track_id
    track.status = status
    track.position.x, track.position.y, track.position.z = position
    track.size.x, track.size.y, track.size.z = size
    return track


def _tracks_array(*tracks):
    array = TrackedObjectArray()
    array.header = _header()
    array.objects = list(tracks)
    return array


def _point(offset_sec, position=(1.0, 0.0, 0.0), variance=0.01):
    point = TrajectoryPoint()
    point.stamp = TimeMsg(sec=0, nanosec=int(round(offset_sec * 1e9)))
    point.position.x, point.position.y, point.position.z = position
    point.covariance = (np.eye(3) * variance).flatten().tolist()
    return point


def _trajectory(track_id=0, points=None):
    trajectory = PredictedTrajectory()
    trajectory.track_id = track_id
    trajectory.model_name = 'constant_velocity'
    trajectory.points = points if points is not None else [_point(0.1)]
    return trajectory


def _trajectories_array(*trajectories):
    array = PredictedTrajectoryArray()
    array.header = _header()
    array.trajectories = list(trajectories)
    return array


def _risk(track_id=0, risk_score=0.5, threat_level=ObstacleRisk.THREAT_MEDIUM):
    risk = ObstacleRisk()
    risk.track_id = track_id
    risk.risk_score = risk_score
    risk.threat_level = threat_level
    return risk


def _risks_array(*risks):
    array = ObstacleRiskArray()
    array.header = _header()
    array.risks = list(risks)
    return array


def test_each_track_produces_a_sphere_and_a_label_marker():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._tracks_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(track_id=1), _track(track_id=2)))

        assert len(published) == 1
        markers = published[0].markers
        assert len(markers) == 4
        namespaces = {marker.ns for marker in markers}
        assert namespaces == {'tracks', 'track_labels'}

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_track_marker_size_floors_at_the_minimum_when_size_is_zero():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._tracks_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(size=(0.0, 0.0, 0.0))))

        sphere = next(m for m in published[0].markers if m.ns == 'tracks')
        assert sphere.scale.x == node._min_size_m
        assert sphere.scale.y == node._min_size_m
        assert sphere.scale.z == node._min_size_m

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_track_status_selects_the_documented_color():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._tracks_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(status=TrackedObject.STATUS_CONFIRMED)))

        sphere = next(m for m in published[0].markers if m.ns == 'tracks')
        assert (sphere.color.r, sphere.color.g, sphere.color.b) == (0.0, 1.0, 0.0)

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_trajectory_produces_a_line_and_an_uncertainty_marker():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._trajectories_callback(_trajectories_array(_trajectory(points=[_point(0.1), _point(0.2)])))

        markers = published[0].markers
        assert len(markers) == 2
        line = next(m for m in markers if m.ns == 'trajectory_lines')
        assert len(line.points) == 2
        uncertainty = next(m for m in markers if m.ns == 'trajectory_uncertainty')
        assert uncertainty.scale.x > 0.0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_trajectory_with_no_points_is_skipped():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._trajectories_callback(_trajectories_array(_trajectory(points=[])))

        assert published[0].markers == []

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_uncertainty_radius_is_clamped_to_the_configured_maximum():
    rclpy.init()
    try:
        node = VisualizationNode()
        node._uncertainty_max_radius_m = 0.5
        published = []
        node._trajectories_pub.publish = published.append

        node._trajectories_callback(_trajectories_array(_trajectory(points=[_point(0.1, variance=1000.0)])))

        uncertainty = next(m for m in published[0].markers if m.ns == 'trajectory_uncertainty')
        assert uncertainty.scale.x <= 1.0  # diameter = 2 * clamped radius

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_risk_with_no_cached_trajectory_is_skipped():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._risks_pub.publish = published.append

        node._risks_callback(_risks_array(_risk(track_id=99)))

        assert published[0].markers == []

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_risk_is_positioned_from_the_matching_cached_trajectory():
    rclpy.init()
    try:
        node = VisualizationNode()
        tracks_published = []
        trajectories_published = []
        risks_published = []
        node._tracks_pub.publish = tracks_published.append
        node._trajectories_pub.publish = trajectories_published.append
        node._risks_pub.publish = risks_published.append

        node._trajectories_callback(_trajectories_array(_trajectory(track_id=5, points=[_point(0.1, position=(3.0, 4.0, 0.0))])))
        node._risks_callback(_risks_array(_risk(track_id=5)))

        sphere = next(m for m in risks_published[0].markers if m.ns == 'risks')
        assert (sphere.pose.position.x, sphere.pose.position.y) == (3.0, 4.0)

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_risk_threat_level_selects_the_documented_color():
    rclpy.init()
    try:
        node = VisualizationNode()
        node._trajectories_callback(_trajectories_array(_trajectory(track_id=1, points=[_point(0.1)])))
        published = []
        node._risks_pub.publish = published.append

        node._risks_callback(_risks_array(_risk(track_id=1, threat_level=ObstacleRisk.THREAT_CRITICAL)))

        sphere = next(m for m in published[0].markers if m.ns == 'risks')
        assert (sphere.color.r, sphere.color.g, sphere.color.b) == (1.0, 0.0, 0.0)

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_header_is_propagated_onto_each_marker():
    rclpy.init()
    try:
        node = VisualizationNode()
        published = []
        node._tracks_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track()))

        assert all(marker.header.frame_id == 'world' for marker in published[0].markers)

        node.destroy_node()
    finally:
        rclpy.shutdown()
