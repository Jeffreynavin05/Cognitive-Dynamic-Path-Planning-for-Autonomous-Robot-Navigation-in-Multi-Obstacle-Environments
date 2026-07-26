"""Constant-velocity motion predictor for Phase 1.

Turns interfaces/TrackedObjectArray (from cognitive_tracking, Module 4) into
interfaces/PredictedTrajectoryArray -- the fixed contract risk_assessment
(Module 6) and visualization_node consume. Any predictor backend (this
project's constant-velocity forecaster, an LSTM, a transformer) must publish
this exact shape so nothing downstream ever has to change; see
interfaces/msg/PredictedTrajectory.msg's own header comment.

Staged the same way tracking_node/perception_node are, so a future backend
swap (e.g. a learned motion model) only ever touches one stage:
    _tracks_callback            -- INPUT.       Entry point; drives every stage
                                    below in order and is the only method
                                    touching the subscriber. Publishes
                                    synchronously once per incoming
                                    TrackedObjectArray -- no separate timer,
                                    mirroring tracking_node's convention (see
                                    this package's README).
    _select_predictable_tracks  -- FILTER.       Only STATUS_CONFIRMED and
                                    STATUS_OCCLUDED tracks are forecast --
                                    STATUS_TENTATIVE is not yet reliable and
                                    STATUS_LOST is about to be dropped by the
                                    tracker.
    _predict_trajectory         -- PREDICT.      Per-track constant-velocity
                                    forward projection
                                    (motion_prediction.trajectory_predictor),
                                    swappable for a learned model later without
                                    touching any other stage.
    _build_trajectory_array     -- ASSEMBLE.     Pure message-building, no I/O
                                    -- directly unit-testable, same pattern as
                                    tracking_node's _build_tracked_array.
    _publish                    -- OUTPUT.       The only method that touches
                                    the publisher.
"""
import numpy as np
import rclpy
from interfaces.msg import (
    PredictedTrajectory,
    PredictedTrajectoryArray,
    TrackedObject,
    TrackedObjectArray,
    TrajectoryPoint,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from motion_prediction.trajectory_predictor import predict_trajectory

TRACKS_TOPIC = '/tracking/tracks'
TRAJECTORIES_TOPIC = '/prediction/trajectories'

MODEL_NAME = 'constant_velocity'

# Only stable tracks are worth forecasting: STATUS_TENTATIVE has too little
# history to trust, and STATUS_LOST is about to be dropped by tracking_node.
PREDICTABLE_STATUSES = (TrackedObject.STATUS_CONFIRMED, TrackedObject.STATUS_OCCLUDED)


def _position_array(track: TrackedObject) -> np.ndarray:
    return np.array([track.position.x, track.position.y, track.position.z])


def _velocity_array(track: TrackedObject) -> np.ndarray:
    return np.array([track.velocity.x, track.velocity.y, track.velocity.z])


class PredictionNode(Node):

    def __init__(self):
        super().__init__('prediction_node')

        self.declare_parameter('horizon_sec', 3.0)
        self.declare_parameter('step_sec', 0.1)
        self.declare_parameter('process_noise_std', 0.5)

        gp = self.get_parameter
        self.horizon_sec = gp('horizon_sec').value
        self.step_sec = gp('step_sec').value
        self.process_noise_std = gp('process_noise_std').value

        # e.g. horizon_sec=3.0 / step_sec=0.1 -> 30 forecast points per track.
        self._num_steps = max(1, round(self.horizon_sec / self.step_sec))

        self._tracks_sub = self.create_subscription(
            TrackedObjectArray, TRACKS_TOPIC, self._tracks_callback, 10)
        self._trajectories_pub = self.create_publisher(
            PredictedTrajectoryArray, TRAJECTORIES_TOPIC, 10)

        self.get_logger().info(
            f'prediction_node ready: subscribed to {TRACKS_TOPIC}, publishing '
            f'PredictedTrajectoryArray on {TRAJECTORIES_TOPIC} synchronously per '
            f'tracking cycle ({self._num_steps} points/track, {self.horizon_sec}s horizon).')

    # ---- input: drives every stage below, in order (Phase 2 backend swap) ----

    def _tracks_callback(self, msg: TrackedObjectArray) -> None:
        predictable = self._select_predictable_tracks(msg.objects)
        trajectories = [self._predict_trajectory(msg.header, track) for track in predictable]

        array = self._build_trajectory_array(msg.header, trajectories)
        self._publish(array)

    # ---- filter (swappable independently of every other stage) ---------------

    def _select_predictable_tracks(self, tracks: list[TrackedObject]) -> list[TrackedObject]:
        return [track for track in tracks if track.status in PREDICTABLE_STATUSES]

    # ---- predict (constant-velocity forecast, swappable for a learned model) -

    def _predict_trajectory(self, header: Header, track: TrackedObject) -> PredictedTrajectory:
        raw_points = predict_trajectory(
            _position_array(track), _velocity_array(track), track.covariance,
            self.process_noise_std, self.step_sec, self._num_steps)

        base_stamp = Time.from_msg(header.stamp)

        trajectory = PredictedTrajectory()
        trajectory.track_id = track.track_id
        trajectory.model_name = MODEL_NAME
        trajectory.points = [
            self._to_trajectory_point(base_stamp, offset, position, velocity, covariance)
            for offset, position, velocity, covariance in raw_points
        ]
        return trajectory

    def _to_trajectory_point(self, base_stamp: Time, offset_sec: float, position: np.ndarray,
                              velocity: np.ndarray, covariance: np.ndarray) -> TrajectoryPoint:
        point = TrajectoryPoint()
        point.stamp = (base_stamp + Duration(seconds=offset_sec)).to_msg()
        point.position.x, point.position.y, point.position.z = position.tolist()
        point.velocity.x, point.velocity.y, point.velocity.z = velocity.tolist()
        point.covariance = covariance.tolist()
        return point

    # ---- assemble (pure, no I/O -- directly unit-testable) ---------------------

    def _build_trajectory_array(self, header: Header,
                                 trajectories: list[PredictedTrajectory]) -> PredictedTrajectoryArray:
        array = PredictedTrajectoryArray()
        array.header.stamp = header.stamp
        array.header.frame_id = header.frame_id
        array.trajectories = trajectories
        return array

    # ---- output -----------------------------------------------------------------

    def _publish(self, array: PredictedTrajectoryArray) -> None:
        self._trajectories_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
