"""Unit tests for prediction_node's filter/predict/assemble pipeline. Doesn't
require a live subscriber/publisher, tracking_node, or Gazebo -- builds
fabricated TrackedObjectArray inputs and drives _tracks_callback directly, the
same direct-method-call pattern test_tracking_node.py uses for tracking_node."""
import numpy as np
import rclpy
from interfaces.msg import TrackedObject, TrackedObjectArray
from std_msgs.msg import Header

from motion_prediction.prediction_node import PredictionNode


def _track(track_id=0, position=(1.0, 2.0, 0.0), velocity=(0.5, 0.0, 0.0),
           status=TrackedObject.STATUS_CONFIRMED, variance=1.0):
    track = TrackedObject()
    track.track_id = track_id
    track.status = status
    track.position.x, track.position.y, track.position.z = position
    track.velocity.x, track.velocity.y, track.velocity.z = velocity
    track.covariance = (np.eye(6) * variance).flatten().tolist()
    return track


def _tracks_array(*tracks):
    array = TrackedObjectArray()
    array.header = Header()
    array.header.frame_id = 'world'
    array.objects = list(tracks)
    return array


def test_confirmed_track_produces_a_trajectory_with_configured_point_count():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(status=TrackedObject.STATUS_CONFIRMED)))

        assert len(published) == 1
        assert len(published[0].trajectories) == 1
        # Default horizon_sec=3.0 / step_sec=0.1 -> 30 points.
        assert len(published[0].trajectories[0].points) == 30

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_occluded_track_is_also_predicted():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(status=TrackedObject.STATUS_OCCLUDED)))

        assert len(published[0].trajectories) == 1

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_tentative_track_is_not_predicted():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(status=TrackedObject.STATUS_TENTATIVE)))

        assert len(published) == 1
        assert published[0].trajectories == []

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_lost_track_is_not_predicted():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(status=TrackedObject.STATUS_LOST)))

        assert len(published) == 1
        assert published[0].trajectories == []

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_trajectory_model_name_is_constant_velocity():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track()))

        assert published[0].trajectories[0].model_name == 'constant_velocity'

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_trajectory_carries_the_source_track_id():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(track_id=7)))

        assert published[0].trajectories[0].track_id == 7

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_first_point_advances_position_by_one_step_of_velocity():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(
            _track(position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0))))

        first_point = published[0].trajectories[0].points[0]
        # step_sec default is 0.1: position should advance by velocity * 0.1.
        assert first_point.position.x == 0.1
        assert first_point.position.y == 0.0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_multiple_confirmed_tracks_each_get_their_own_trajectory():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track(track_id=1), _track(track_id=2)))

        assert len(published[0].trajectories) == 2
        assert {t.track_id for t in published[0].trajectories} == {1, 2}

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_is_called_synchronously_once_per_tracks_array():
    rclpy.init()
    try:
        node = PredictionNode()
        published = []
        node._trajectories_pub.publish = published.append

        node._tracks_callback(_tracks_array(_track()))
        node._tracks_callback(_tracks_array(_track()))

        assert len(published) == 2

        node.destroy_node()
    finally:
        rclpy.shutdown()
