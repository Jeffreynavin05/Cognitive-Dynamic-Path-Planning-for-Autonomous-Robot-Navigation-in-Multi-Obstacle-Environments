"""Unit tests for tracking_node's association/Kalman/lifecycle pipeline. Doesn't
require a live subscriber/publisher or Gazebo -- builds fabricated
DetectedObjectArray inputs and drives _detections_callback directly, the same
direct-method-call pattern test_perception_node.py uses for perception_node."""
import rclpy
from interfaces.msg import DetectedObject, DetectedObjectArray, TrackedObject
from std_msgs.msg import Header

from cognitive_tracking.tracking_node import TrackingNode


def _detection(x, y, z=0.0, class_id=DetectedObject.CLASS_STATIC_OBSTACLE):
    detection = DetectedObject()
    detection.class_id = class_id
    detection.position.x, detection.position.y, detection.position.z = x, y, z
    detection.size.x = detection.size.y = detection.size.z = 0.4
    detection.confidence = 1.0
    return detection


def _detections_array(*detections):
    array = DetectedObjectArray()
    array.header = Header()
    array.header.frame_id = 'world'
    array.objects = list(detections)
    return array


def test_unmatched_detection_spawns_a_new_tentative_track():
    rclpy.init()
    try:
        node = TrackingNode()

        node._detections_callback(_detections_array(_detection(1.0, 2.0)))

        assert len(node._tracks) == 1
        track = node._tracks[0]
        assert track.status == TrackedObject.STATUS_TENTATIVE
        assert track.hits == 1

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_track_confirms_after_three_consecutive_hits():
    rclpy.init()
    try:
        node = TrackingNode()

        for _ in range(3):
            node._detections_callback(_detections_array(_detection(1.0, 2.0)))

        assert len(node._tracks) == 1
        assert node._tracks[0].status == TrackedObject.STATUS_CONFIRMED
        assert node._tracks[0].hits == 3

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_same_detection_across_cycles_keeps_the_same_track_id():
    rclpy.init()
    try:
        node = TrackingNode()

        node._detections_callback(_detections_array(_detection(1.0, 2.0)))
        first_id = node._tracks[0].track_id
        node._detections_callback(_detections_array(_detection(1.05, 2.0)))  # small move, within gate

        assert len(node._tracks) == 1
        assert node._tracks[0].track_id == first_id

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_missed_confirmed_track_becomes_occluded_then_lost_and_is_dropped():
    rclpy.init()
    try:
        node = TrackingNode()

        for _ in range(3):
            node._detections_callback(_detections_array(_detection(1.0, 2.0)))
        assert node._tracks[0].status == TrackedObject.STATUS_CONFIRMED

        node._detections_callback(_detections_array())  # no detections: 1 miss
        assert node._tracks[0].status == TrackedObject.STATUS_OCCLUDED

        for _ in range(4):  # 4 more consecutive misses = 5 total => lost_after_misses
            node._detections_callback(_detections_array())

        assert len(node._tracks) == 0  # LOST tracks are pruned after being published once

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_sends_a_tracked_object_array_synchronously():
    rclpy.init()
    try:
        node = TrackingNode()
        published = []
        node._tracks_pub.publish = published.append

        node._detections_callback(_detections_array(_detection(1.0, 2.0)))

        assert len(published) == 1
        assert len(published[0].objects) == 1
        assert published[0].objects[0].class_id == DetectedObject.CLASS_STATIC_OBSTACLE

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_two_far_apart_detections_spawn_two_distinct_tracks():
    rclpy.init()
    try:
        node = TrackingNode()

        node._detections_callback(_detections_array(_detection(0.0, 0.0), _detection(10.0, 10.0)))

        assert len(node._tracks) == 2
        assert node._tracks[0].track_id != node._tracks[1].track_id

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_class_id_updates_from_newest_matched_detection():
    rclpy.init()
    try:
        node = TrackingNode()

        node._detections_callback(_detections_array(
            _detection(1.0, 2.0, class_id=DetectedObject.CLASS_STATIC_OBSTACLE)))
        node._detections_callback(_detections_array(
            _detection(1.0, 2.0, class_id=DetectedObject.CLASS_DYNAMIC_OBSTACLE)))

        assert len(node._tracks) == 1
        assert node._tracks[0].class_id == DetectedObject.CLASS_DYNAMIC_OBSTACLE

        node.destroy_node()
    finally:
        rclpy.shutdown()
