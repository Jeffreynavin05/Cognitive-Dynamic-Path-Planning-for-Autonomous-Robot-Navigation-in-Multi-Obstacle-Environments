"""Ground-truth object detector for Phase 1.

Turns Gazebo's ground-truth entity poses into interfaces/DetectedObjectArray, the
fixed contract cognitive_tracking (Module 4) consumes. This is deliberately the
*only* node that reads the sim-only obstacle pose topics: each obstacle spawns with
its own PosePublisher plugin (simulation_manager.OBSTACLE_POSE_PUBLISHER_PLUGIN),
bridged per-model onto `/model/<name>/pose` (dynamic) or `/model/<name>/pose_static`
(static) -- see _obstacle_pose_topics() for why one topic per obstacle is needed
instead of a single shared one, and why plain /world/<world_name>/pose/info can't be
used at all: SceneBroadcaster's Pose_V never sets the header data the tf2_msgs bridge
conversion needs, so child_frame_id comes through empty -- on the real BeetleBot
(Phase 2), a camera/lidar-based detector backend replaces this node's internals while
publishing the exact same message shape on the same /perception/detections topic, so
nothing downstream ever has to change.

Kept deliberately staged rather than one callback that does everything, so a Phase-2
swap only ever touches one stage:
  _pose_callback          -- INPUT.  Phase 1: caches ground-truth poses. Phase 2:
                              replaced by whatever camera_node/lidar_node feed in.
  classify /
  _filter_known_entities  -- FILTER. Decides which raw inputs are real obstacles.
  _to_detected_object     -- CONVERT. One input -> one DetectedObject.
  _build_detection_array  -- ASSEMBLE. Pure message-building, no I/O, unit-testable.
  _publish_detections     -- OUTPUT. The only method that touches the publisher.
"""
import rclpy
from geometry_msgs.msg import Transform
from interfaces.msg import DetectedObject, DetectedObjectArray
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

DETECTIONS_TOPIC = '/perception/detections'


def _obstacle_pose_topics(num_static: int, num_dynamic: int) -> list[str]:
    """Every obstacle's own PosePublisher topic (see simulation_manager's
    static_obstacle_pose_topic()/dynamic_obstacle_pose_topic(), re-derived here rather
    than imported since perception_node has no build dependency on simulation).
    Confirmed on a real Harmonic install: this build's PosePublisher ignores a custom
    <topic> override and always publishes per-model, with a static_publisher entity
    only ever appearing on its "..._static"-suffixed topic -- so one shared topic
    isn't possible and num_static_obstacles/num_dynamic_obstacles (hand-duplicated
    from simulation/config/simulation_params.yaml, not derived) are needed just to
    know how many per-model topics to subscribe to."""
    return (
        [f'/model/static_obstacle_{i}/pose_static' for i in range(num_static)]
        + [f'/model/dynamic_obstacle_{i}/pose' for i in range(num_dynamic)]
    )


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('detection_frame_id', 'world')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('static_obstacle_size', 0.45)
        self.declare_parameter('dynamic_obstacle_diameter', 0.4)
        self.declare_parameter('dynamic_obstacle_height', 0.3)
        self.declare_parameter('num_static_obstacles', 4)
        self.declare_parameter('num_dynamic_obstacles', 6)

        gp = self.get_parameter
        self.frame_id = gp('detection_frame_id').value
        self.publish_rate = gp('publish_rate_hz').value
        self.static_size = gp('static_obstacle_size').value
        self.dynamic_diameter = gp('dynamic_obstacle_diameter').value
        self.dynamic_height = gp('dynamic_obstacle_height').value

        # Latest known transform per Gazebo entity name. A plain dict is safe here
        # without a lock: rclpy's default single-threaded executor runs the pose
        # callback and the publish timer on the same thread, never concurrently.
        self._latest_poses: dict[str, Transform] = {}

        pose_topics = _obstacle_pose_topics(
            gp('num_static_obstacles').value, gp('num_dynamic_obstacles').value)
        self._pose_subs = [
            self.create_subscription(TFMessage, topic, self._pose_callback, 10)
            for topic in pose_topics
        ]
        self._detections_pub = self.create_publisher(DetectedObjectArray, DETECTIONS_TOPIC, 10)
        self._timer = self.create_timer(1.0 / self.publish_rate, self._publish_detections)

        self.get_logger().info(
            f'perception_node ready: ground-truth detector subscribed to {len(pose_topics)} '
            f'obstacle pose topics, publishing DetectedObjectArray on {DETECTIONS_TOPIC} '
            f'at {self.publish_rate} Hz.')

    # ---- input (Phase 2 replaces this) -------------------------------------

    def _pose_callback(self, msg: TFMessage) -> None:
        """Detection source, Phase 1: cache the latest ground-truth transform per
        Gazebo entity. This is the only method a Phase-2 camera/lidar detection
        source would need to stop calling / replace -- everything below reads
        from self._latest_poses without caring how it got populated."""
        for transform_stamped in msg.transforms:
            self._latest_poses[transform_stamped.child_frame_id] = transform_stamped.transform

    # ---- filter -------------------------------------------------------------

    def classify(self, entity_name: str):
        """Map a Gazebo entity name to (class_id, (size_x, size_y, size_z)), or None
        if the entity isn't an obstacle simulation_manager spawned -- e.g. the robot
        itself, arena walls, or the ground plane. Matches the static_obstacle_* /
        dynamic_obstacle_* naming convention documented in simulation/README.md, so
        anything else is deliberately not reported as a detection at all (not even
        CLASS_UNKNOWN) rather than guessing at what it might be."""
        if entity_name.startswith('static_obstacle_'):
            return DetectedObject.CLASS_STATIC_OBSTACLE, (self.static_size, self.static_size, self.static_size)
        if entity_name.startswith('dynamic_obstacle_'):
            return (DetectedObject.CLASS_DYNAMIC_OBSTACLE,
                    (self.dynamic_diameter, self.dynamic_diameter, self.dynamic_height))
        return None

    def _filter_known_entities(self) -> list[tuple[Transform, int, tuple[float, float, float]]]:
        """Reduce every currently-cached pose down to the ones classify() recognizes
        as an obstacle, discarding the rest (robot, walls, ground plane, ...).
        Returns (transform, class_id, size) tuples ready for _to_detected_object."""
        filtered = []
        for entity_name, transform in self._latest_poses.items():
            classification = self.classify(entity_name)
            if classification is None:
                continue
            class_id, size = classification
            filtered.append((transform, class_id, size))
        return filtered

    # ---- convert --------------------------------------------------------------

    def _to_detected_object(self, header: Header, transform: Transform, class_id: int,
                             size: tuple[float, float, float]) -> DetectedObject:
        """Convert one filtered ground-truth pose into a single DetectedObject. This
        is the method a Phase-2 detector backend replaces internally (e.g. reading a
        bounding box off an image instead of a config-approximated size) while
        keeping the exact same DetectedObject shape everything downstream expects."""
        detection = DetectedObject()
        detection.header = header
        detection.class_id = class_id
        detection.position.x = transform.translation.x
        detection.position.y = transform.translation.y
        detection.position.z = transform.translation.z
        detection.size.x, detection.size.y, detection.size.z = size
        # Ground truth is exact, not a model estimate, so confidence is pinned to 1.0
        # for this node's entire Phase-1 lifetime -- there's no detector uncertainty
        # to report when the "detector" is just reading Gazebo's own state. The field
        # still exists on DetectedObject (and is populated here, not left at its
        # zero-value default) so that nothing downstream special-cases "ground truth
        # mode": Phase 2 swaps this line for the real detector's per-object confidence
        # (e.g. a YOLO score or LiDAR cluster fit quality) and everything else is
        # already built to consume a variable confidence value.
        detection.confidence = 1.0
        return detection

    # ---- assemble (pure, no I/O -- directly unit-testable) ---------------------

    def _build_detection_array(self) -> DetectedObjectArray:
        array = DetectedObjectArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.frame_id
        array.objects = [
            self._to_detected_object(array.header, transform, class_id, size)
            for transform, class_id, size in self._filter_known_entities()
        ]
        return array

    # ---- output ---------------------------------------------------------------

    def _publish_detections(self) -> None:
        """Timer callback: the only method that touches the publisher. Swapping the
        detection source (Phase 2) never requires touching this method."""
        self._detections_pub.publish(self._build_detection_array())


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
