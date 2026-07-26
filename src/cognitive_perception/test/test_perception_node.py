"""Unit tests for perception_node's ground-truth classification/publish logic.
Doesn't touch Gazebo -- exercises the pose cache and message-building methods
directly, the same direct-method-call pattern simulation/test/test_geometry.py
uses for simulation_manager. rclpy.init()/shutdown() are called manually per
test since there's no test fixture framework in use yet in this repo."""
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from interfaces.msg import DetectedObject
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from cognitive_perception.perception_node import PerceptionNode


def _transform_stamped(name: str, x: float, y: float, z: float) -> TransformStamped:
    t = TransformStamped()
    t.child_frame_id = name
    t.transform = Transform()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    return t


def test_classify_maps_known_prefixes_and_ignores_everything_else():
    rclpy.init()
    try:
        node = PerceptionNode()

        static_class, static_size = node.classify('static_obstacle_2')
        assert static_class == DetectedObject.CLASS_STATIC_OBSTACLE
        assert static_size == (node.static_size, node.static_size, node.static_size)

        dynamic_class, dynamic_size = node.classify('dynamic_obstacle_5')
        assert dynamic_class == DetectedObject.CLASS_DYNAMIC_OBSTACLE
        assert dynamic_size == (node.dynamic_diameter, node.dynamic_diameter, node.dynamic_height)

        # The robot itself, arena walls, and the ground plane must never become
        # detections -- only the static_obstacle_*/dynamic_obstacle_* prefixes count.
        assert node.classify('beetlebot') is None
        assert node.classify('ground_plane') is None

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_pose_callback_caches_latest_transform_per_entity():
    rclpy.init()
    try:
        node = PerceptionNode()

        node._pose_callback(TFMessage(transforms=[_transform_stamped('static_obstacle_0', 1.5, -2.0, 0.05)]))
        assert node._latest_poses['static_obstacle_0'].translation.x == 1.5

        # A second message for the same entity overwrites, it doesn't accumulate.
        node._pose_callback(TFMessage(transforms=[_transform_stamped('static_obstacle_0', 3.0, 1.0, 0.05)]))
        assert len(node._latest_poses) == 1
        assert node._latest_poses['static_obstacle_0'].translation.x == 3.0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_filter_known_entities_drops_unrecognized_names():
    rclpy.init()
    try:
        node = PerceptionNode()

        node._pose_callback(TFMessage(transforms=[
            _transform_stamped('static_obstacle_0', 1.0, 2.0, 0.05),
            _transform_stamped('dynamic_obstacle_3', -1.0, 0.5, 0.05),
            _transform_stamped('beetlebot', 0.0, 0.0, 0.0),
            _transform_stamped('ground_plane', 0.0, 0.0, 0.0),
        ]))

        filtered = node._filter_known_entities()

        # Only the two spawned obstacles survive the filter stage; the robot and
        # ground plane are dropped before conversion ever sees them.
        assert len(filtered) == 2
        class_ids = {class_id for _transform, class_id, _size in filtered}
        assert class_ids == {DetectedObject.CLASS_STATIC_OBSTACLE, DetectedObject.CLASS_DYNAMIC_OBSTACLE}

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_to_detected_object_converts_pose_and_pins_ground_truth_confidence():
    rclpy.init()
    try:
        node = PerceptionNode()

        header = Header()
        header.frame_id = 'world'
        transform = _transform_stamped('static_obstacle_0', 1.0, 2.0, 0.05).transform
        size = (node.static_size, node.static_size, node.static_size)

        detection = node._to_detected_object(header, transform, DetectedObject.CLASS_STATIC_OBSTACLE, size)

        assert detection.header is header
        assert detection.class_id == DetectedObject.CLASS_STATIC_OBSTACLE
        assert (detection.position.x, detection.position.y, detection.position.z) == (1.0, 2.0, 0.05)
        assert detection.size.x == node.static_size
        # Ground truth is exact -- confidence is always 1.0 in Phase 1. A real
        # detector backend (Phase 2) would set this to its own per-object score.
        assert detection.confidence == 1.0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_build_detection_array_is_pure_and_matches_filtered_entities():
    rclpy.init()
    try:
        node = PerceptionNode()
        node._pose_callback(TFMessage(transforms=[
            _transform_stamped('static_obstacle_0', 1.0, 2.0, 0.05),
            _transform_stamped('beetlebot', 0.0, 0.0, 0.0),
        ]))

        array = node._build_detection_array()

        assert len(array.objects) == 1  # beetlebot filtered out, only the obstacle remains
        assert array.header.frame_id == node.frame_id
        assert array.objects[0].class_id == DetectedObject.CLASS_STATIC_OBSTACLE
        assert array.objects[0].position.x == 1.0
        assert array.objects[0].confidence == 1.0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_detections_publishes_the_built_array():
    rclpy.init()
    try:
        node = PerceptionNode()
        published = []
        node._detections_pub.publish = published.append

        node._pose_callback(TFMessage(transforms=[_transform_stamped('dynamic_obstacle_1', -2.0, 0.5, 0.05)]))
        node._publish_detections()

        assert len(published) == 1
        assert published[0].objects[0].class_id == DetectedObject.CLASS_DYNAMIC_OBSTACLE

        node.destroy_node()
    finally:
        rclpy.shutdown()
