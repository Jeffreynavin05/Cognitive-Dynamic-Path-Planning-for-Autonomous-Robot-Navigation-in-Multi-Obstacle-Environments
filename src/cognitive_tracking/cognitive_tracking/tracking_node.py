"""Kalman-filter multi-object tracker for Phase 1.

Turns interfaces/DetectedObjectArray (from cognitive_perception, Module 3) into
interfaces/TrackedObjectArray -- the fixed contract motion_prediction (Module 5)
and visualization_node consume. Any tracker backend (this project's Kalman filter,
DeepSORT, ByteTrack) must publish this exact shape so nothing downstream ever has
to change; see interfaces/msg/TrackedObject.msg's own header comment.

Staged the same way perception_node is, so a future backend swap (e.g. DeepSORT's
appearance-based association, or a learned motion model) only ever touches one
stage:
    _detections_callback  -- INPUT.        Entry point; drives every stage below in
                              order and is the only method touching the subscriber.
                              Publishes synchronously once per incoming
                              DetectedObjectArray -- no separate prediction timer
                              in Phase 1 (see this package's README).
    _associate            -- ASSOCIATION.  Greedy nearest-neighbor over a Euclidean
                              cost matrix (cognitive_tracking.association), matched
                              against each track's pre-predict position. Swappable
                              for Hungarian assignment later without touching any
                              other stage.
    _predict              -- KALMAN PREDICT. Every existing track is coasted
                              forward by dt, matched or not.
    _update               -- KALMAN UPDATE.  Only matched tracks are corrected with
                              their associated detection's position.
    _manage_lifecycle     -- LIFECYCLE.     Drives each Track's STATUS_* state
                              machine (cognitive_tracking.track.Track) and spawns
                              new tentative tracks for unmatched detections.
    _build_tracked_array  -- ASSEMBLE.      Pure message-building, no I/O --
                              directly unit-testable, same pattern as
                              perception_node's _build_detection_array.
    _publish              -- OUTPUT.        The only method that touches the
                              publisher.
"""
import numpy as np
import rclpy
from interfaces.msg import DetectedObject, DetectedObjectArray, TrackedObject, TrackedObjectArray
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from cognitive_tracking.association import build_cost_matrix, greedy_nearest_neighbor
from cognitive_tracking.kalman_filter import KalmanFilter6D
from cognitive_tracking.track import Track

DETECTIONS_TOPIC = '/perception/detections'
TRACKS_TOPIC = '/tracking/tracks'


def _position_array(detection: DetectedObject) -> np.ndarray:
    return np.array([detection.position.x, detection.position.y, detection.position.z])


class TrackingNode(Node):

    def __init__(self):
        super().__init__('tracking_node')

        self.declare_parameter('gating_distance_m', 1.0)
        self.declare_parameter('confirm_after_hits', 3)
        self.declare_parameter('occluded_after_misses', 1)
        self.declare_parameter('lost_after_misses', 5)
        self.declare_parameter('process_noise_std', 0.5)
        self.declare_parameter('measurement_noise_std', 0.1)
        self.declare_parameter('initial_velocity_variance', 10.0)
        self.declare_parameter('min_dt_sec', 0.001)

        gp = self.get_parameter
        self.gating_distance = gp('gating_distance_m').value
        self.confirm_after_hits = gp('confirm_after_hits').value
        self.occluded_after_misses = gp('occluded_after_misses').value
        self.lost_after_misses = gp('lost_after_misses').value
        self.process_noise_std = gp('process_noise_std').value
        self.measurement_noise_std = gp('measurement_noise_std').value
        self.initial_velocity_variance = gp('initial_velocity_variance').value
        self.min_dt = gp('min_dt_sec').value

        self._tracks: list[Track] = []
        self._next_track_id = 0
        self._last_stamp: Time | None = None

        self._detections_sub = self.create_subscription(
            DetectedObjectArray, DETECTIONS_TOPIC, self._detections_callback, 10)
        self._tracks_pub = self.create_publisher(TrackedObjectArray, TRACKS_TOPIC, 10)

        self.get_logger().info(
            f'tracking_node ready: subscribed to {DETECTIONS_TOPIC}, publishing '
            f'TrackedObjectArray on {TRACKS_TOPIC} synchronously per detection cycle.')

    # ---- input: drives every stage below, in order (Phase 2 backend swap) ----

    def _detections_callback(self, msg: DetectedObjectArray) -> None:
        dt = self._compute_dt(msg.header)

        matches, unmatched_tracks, unmatched_detections = self._associate(msg.objects)
        self._predict(dt)
        self._update(matches, msg.objects)
        self._manage_lifecycle(matches, unmatched_tracks, unmatched_detections, msg.objects)

        array = self._build_tracked_array(msg.header)
        self._publish(array)

        # Tracks the array above just reported as LOST are removed only *after*
        # publishing, so STATUS_LOST is observed by consumers at least once (see
        # TrackedObject.msg: "about to be dropped by the tracker") instead of a
        # track disappearing from the topic with no terminal status ever seen.
        self._prune_lost_tracks()

    def _compute_dt(self, header: Header) -> float:
        stamp = Time.from_msg(header.stamp)
        if self._last_stamp is None:
            dt = self.min_dt
        else:
            # Clamped rather than left possibly zero/negative (duplicate or
            # out-of-order stamps) -- the constant-velocity predict step would
            # otherwise walk state backward or do a no-op that silently hides a
            # timestamp problem.
            dt = max((stamp - self._last_stamp).nanoseconds / 1e9, self.min_dt)
        self._last_stamp = stamp
        return dt

    # ---- association (swappable independently of every other stage) ----------

    def _associate(self, detections: list[DetectedObject]):
        track_positions = np.array([track.kf.position for track in self._tracks])
        detection_positions = np.array([_position_array(detection) for detection in detections])
        cost_matrix = build_cost_matrix(track_positions, detection_positions)
        return greedy_nearest_neighbor(cost_matrix, self.gating_distance)

    # ---- kalman predict (all tracks, matched or not) --------------------------

    def _predict(self, dt: float) -> None:
        for track in self._tracks:
            track.kf.predict(dt)

    # ---- kalman update (matched tracks only) ----------------------------------

    def _update(self, matches: list[tuple[int, int]], detections: list[DetectedObject]) -> None:
        for track_idx, detection_idx in matches:
            self._tracks[track_idx].kf.update(_position_array(detections[detection_idx]))

    # ---- lifecycle management --------------------------------------------------

    def _manage_lifecycle(self, matches: list[tuple[int, int]], unmatched_tracks: list[int],
                           unmatched_detections: list[int], detections: list[DetectedObject]) -> None:
        for track_idx, detection_idx in matches:
            detection = detections[detection_idx]
            size = (detection.size.x, detection.size.y, detection.size.z)
            self._tracks[track_idx].register_hit(detection.class_id, size, self.confirm_after_hits)

        for track_idx in unmatched_tracks:
            self._tracks[track_idx].register_miss(self.occluded_after_misses, self.lost_after_misses)

        for detection_idx in unmatched_detections:
            self._spawn_track(detections[detection_idx])

    def _spawn_track(self, detection: DetectedObject) -> None:
        kf = KalmanFilter6D(
            initial_position=_position_array(detection),
            process_noise_std=self.process_noise_std,
            measurement_noise_std=self.measurement_noise_std,
            initial_velocity_variance=self.initial_velocity_variance)
        size = (detection.size.x, detection.size.y, detection.size.z)
        track = Track(track_id=self._next_track_id, kf=kf, class_id=detection.class_id, size=size)
        self._next_track_id += 1
        self._tracks.append(track)

    def _prune_lost_tracks(self) -> None:
        self._tracks = [track for track in self._tracks if track.status != TrackedObject.STATUS_LOST]

    # ---- assemble (pure, no I/O -- directly unit-testable) ---------------------

    def _build_tracked_array(self, header: Header) -> TrackedObjectArray:
        array = TrackedObjectArray()
        array.header.stamp = header.stamp
        array.header.frame_id = header.frame_id
        array.objects = [self._to_tracked_object(array.header, track) for track in self._tracks]
        return array

    def _to_tracked_object(self, header: Header, track: Track) -> TrackedObject:
        tracked = TrackedObject()
        tracked.header = header
        tracked.track_id = track.track_id
        tracked.class_id = track.class_id
        tracked.position.x, tracked.position.y, tracked.position.z = track.kf.position.tolist()
        tracked.velocity.x, tracked.velocity.y, tracked.velocity.z = track.kf.velocity.tolist()
        tracked.size.x, tracked.size.y, tracked.size.z = track.size
        tracked.covariance = track.kf.covariance_flat.tolist()
        tracked.status = track.status
        tracked.age = track.age
        return tracked

    # ---- output -----------------------------------------------------------------

    def _publish(self, array: TrackedObjectArray) -> None:
        self._tracks_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
