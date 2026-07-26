"""Debugging visualization for Phase 1. Not part of the control path.

Independently subscribes to interfaces/TrackedObjectArray,
interfaces/PredictedTrajectoryArray, and interfaces/ObstacleRiskArray --
exactly the precedented "independent multi-topic subscription" pattern
already documented for this node in PROJECT_CONTEXT.md sections 6/9c/15 --
and turns each into its own visualization_msgs/MarkerArray for RViz. Three
separate output topics (not one combined array) so each pipeline stage can be
toggled independently in RViz's Displays panel.

Staged the same way every other node in this workspace is (PROJECT_CONTEXT.md
section 10), one input/assemble/output triple per subscribed topic:
    _tracks_callback         -- INPUT/ASSEMBLE/OUTPUT for /tracking/tracks.
    _trajectories_callback   -- INPUT/ASSEMBLE/OUTPUT for
                                 /prediction/trajectories. Also caches the
                                 latest trajectory per track_id (world-frame
                                 position source for risk markers -- see
                                 below).
    _risks_callback          -- INPUT/ASSEMBLE/OUTPUT for
                                 /risk/obstacle_risks.
    _build_track_markers / _build_trajectory_markers / _build_risk_markers
                              -- ASSEMBLE. Pure message-building, no I/O --
                                 directly unit-testable.
    _publish_tracks / _publish_trajectories / _publish_risks
                              -- OUTPUT. The only methods that touch their
                                 respective publisher.

interfaces/msg/ObstacleRisk.msg carries no position field of its own (its own
header comment: "not a raw sensor measurement"), so a risk marker's position
is read from the matching track_id's most recently cached
PredictedTrajectory's first point instead -- the same by-track_id join
dynamic_planner's planner_node already uses to combine trajectory geometry
with risk priority (PROJECT_CONTEXT.md section 9c/16), and the same
"proxy for a true t=0 sample" documented for risk_node's own
distance_to_robot/relative_speed (section 9b). A risk with no matching cached
trajectory that cycle is silently skipped, precedented by planner_node's
_join_obstacles doing the same (section 14).
"""
import numpy as np
import rclpy
from interfaces.msg import ObstacleRisk, ObstacleRiskArray, PredictedTrajectory, PredictedTrajectoryArray, \
    TrackedObject, TrackedObjectArray
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

TRACKS_TOPIC = '/tracking/tracks'
TRAJECTORIES_TOPIC = '/prediction/trajectories'
RISKS_TOPIC = '/risk/obstacle_risks'

TRACK_MARKERS_TOPIC = '/visualization/tracks_markers'
TRAJECTORY_MARKERS_TOPIC = '/visualization/trajectories_markers'
RISK_MARKERS_TOPIC = '/visualization/risk_markers'

# TrackedObject.status -> RGBA. Mirrors interfaces/msg/TrackedObject.msg's own
# STATUS_* ordering.
_TRACK_STATUS_COLOR = {
    TrackedObject.STATUS_TENTATIVE: (1.0, 1.0, 0.0, 0.8),
    TrackedObject.STATUS_CONFIRMED: (0.0, 1.0, 0.0, 0.8),
    TrackedObject.STATUS_OCCLUDED: (1.0, 0.5, 0.0, 0.8),
    TrackedObject.STATUS_LOST: (0.5, 0.5, 0.5, 0.8),
}
_TRACK_STATUS_NAME = {
    TrackedObject.STATUS_TENTATIVE: 'TENTATIVE',
    TrackedObject.STATUS_CONFIRMED: 'CONFIRMED',
    TrackedObject.STATUS_OCCLUDED: 'OCCLUDED',
    TrackedObject.STATUS_LOST: 'LOST',
}

# ObstacleRisk.threat_level -> RGB (alpha is scaled separately by risk_score).
_THREAT_COLOR = {
    ObstacleRisk.THREAT_LOW: (0.0, 1.0, 0.0),
    ObstacleRisk.THREAT_MEDIUM: (1.0, 1.0, 0.0),
    ObstacleRisk.THREAT_HIGH: (1.0, 0.5, 0.0),
    ObstacleRisk.THREAT_CRITICAL: (1.0, 0.0, 0.0),
}
_THREAT_NAME = {
    ObstacleRisk.THREAT_LOW: 'LOW',
    ObstacleRisk.THREAT_MEDIUM: 'MEDIUM',
    ObstacleRisk.THREAT_HIGH: 'HIGH',
    ObstacleRisk.THREAT_CRITICAL: 'CRITICAL',
}

_TRAJECTORY_LINE_COLOR = (0.0, 1.0, 1.0, 0.6)
_TRAJECTORY_UNCERTAINTY_COLOR = (0.6, 0.6, 0.6, 0.25)


class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')

        self.declare_parameter('track_marker_min_size_m', 0.2)
        self.declare_parameter('track_label_height_m', 0.3)
        self.declare_parameter('trajectory_line_width_m', 0.05)
        self.declare_parameter('trajectory_uncertainty_max_radius_m', 2.0)
        self.declare_parameter('risk_marker_diameter_m', 0.6)
        self.declare_parameter('marker_lifetime_sec', 1.0)

        gp = self.get_parameter
        self._min_size_m = gp('track_marker_min_size_m').value
        self._label_height_m = gp('track_label_height_m').value
        self._line_width_m = gp('trajectory_line_width_m').value
        self._uncertainty_max_radius_m = gp('trajectory_uncertainty_max_radius_m').value
        self._risk_diameter_m = gp('risk_marker_diameter_m').value
        self._lifetime = Duration(seconds=gp('marker_lifetime_sec').value)

        # Latest PredictedTrajectory per track_id -- the only state this node
        # keeps, purely to give risk markers a position (see module docstring).
        self._latest_trajectories: dict[int, PredictedTrajectory] = {}

        self._tracks_sub = self.create_subscription(TrackedObjectArray, TRACKS_TOPIC, self._tracks_callback, 10)
        self._trajectories_sub = self.create_subscription(
            PredictedTrajectoryArray, TRAJECTORIES_TOPIC, self._trajectories_callback, 10)
        self._risks_sub = self.create_subscription(ObstacleRiskArray, RISKS_TOPIC, self._risks_callback, 10)

        self._tracks_pub = self.create_publisher(MarkerArray, TRACK_MARKERS_TOPIC, 10)
        self._trajectories_pub = self.create_publisher(MarkerArray, TRAJECTORY_MARKERS_TOPIC, 10)
        self._risks_pub = self.create_publisher(MarkerArray, RISK_MARKERS_TOPIC, 10)

        self.get_logger().info(
            f'visualization_node ready: {TRACKS_TOPIC} -> {TRACK_MARKERS_TOPIC}, '
            f'{TRAJECTORIES_TOPIC} -> {TRAJECTORY_MARKERS_TOPIC}, {RISKS_TOPIC} -> {RISK_MARKERS_TOPIC}.')

    # ---- tracks: input/assemble/output ------------------------------------------

    def _tracks_callback(self, msg: TrackedObjectArray) -> None:
        self._publish_tracks(self._build_track_markers(msg))

    def _build_track_markers(self, msg: TrackedObjectArray) -> MarkerArray:
        array = MarkerArray()
        for track in msg.objects:
            array.markers.append(self._track_sphere(msg.header, track))
            array.markers.append(self._track_label(msg.header, track))
        return array

    def _track_sphere(self, header: Header, track: TrackedObject) -> Marker:
        color = _TRACK_STATUS_COLOR[track.status]
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'tracks'
        marker.id = track.track_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = track.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(track.size.x, self._min_size_m)
        marker.scale.y = max(track.size.y, self._min_size_m)
        marker.scale.z = max(track.size.z, self._min_size_m)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = self._lifetime.to_msg()
        return marker

    def _track_label(self, header: Header, track: TrackedObject) -> Marker:
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'track_labels'
        marker.id = track.track_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = track.position.x
        marker.pose.position.y = track.position.y
        marker.pose.position.z = track.position.z + max(track.size.z, self._min_size_m) / 2.0 + self._label_height_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._label_height_m
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'id={track.track_id} {_TRACK_STATUS_NAME[track.status]}'
        marker.lifetime = self._lifetime.to_msg()
        return marker

    def _publish_tracks(self, array: MarkerArray) -> None:
        self._tracks_pub.publish(array)

    # ---- trajectories: input/assemble/output, plus the risk-marker cache -------

    def _trajectories_callback(self, msg: PredictedTrajectoryArray) -> None:
        self._latest_trajectories = {trajectory.track_id: trajectory for trajectory in msg.trajectories}
        self._publish_trajectories(self._build_trajectory_markers(msg))

    def _build_trajectory_markers(self, msg: PredictedTrajectoryArray) -> MarkerArray:
        array = MarkerArray()
        for trajectory in msg.trajectories:
            if not trajectory.points:
                continue
            array.markers.append(self._trajectory_line(msg.header, trajectory))
            array.markers.append(self._trajectory_uncertainty(msg.header, trajectory))
        return array

    def _trajectory_line(self, header: Header, trajectory: PredictedTrajectory) -> Marker:
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'trajectory_lines'
        marker.id = trajectory.track_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._line_width_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = _TRAJECTORY_LINE_COLOR
        marker.points = [point.position for point in trajectory.points]
        marker.lifetime = self._lifetime.to_msg()
        return marker

    def _trajectory_uncertainty(self, header: Header, trajectory: PredictedTrajectory) -> Marker:
        final_point = trajectory.points[-1]
        covariance = np.array(final_point.covariance).reshape(3, 3)
        # sqrt(mean variance) as a single representative std-dev radius --
        # deterministic, no distributional assumption beyond what the
        # trajectory's own covariance already encodes (same closed-form
        # spirit as risk_model.py's Gaussian falloff, section 9b).
        radius = min(float(np.sqrt(max(np.trace(covariance) / 3.0, 0.0))), self._uncertainty_max_radius_m)

        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'trajectory_uncertainty'
        marker.id = trajectory.track_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = final_point.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = max(radius * 2.0, self._min_size_m)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = _TRAJECTORY_UNCERTAINTY_COLOR
        marker.lifetime = self._lifetime.to_msg()
        return marker

    def _publish_trajectories(self, array: MarkerArray) -> None:
        self._trajectories_pub.publish(array)

    # ---- risks: input/assemble/output -------------------------------------------

    def _risks_callback(self, msg: ObstacleRiskArray) -> None:
        self._publish_risks(self._build_risk_markers(msg))

    def _build_risk_markers(self, msg: ObstacleRiskArray) -> MarkerArray:
        array = MarkerArray()
        for risk in msg.risks:
            trajectory = self._latest_trajectories.get(risk.track_id)
            if trajectory is None or not trajectory.points:
                continue
            position = trajectory.points[0].position
            array.markers.append(self._risk_sphere(msg.header, risk, position))
            array.markers.append(self._risk_label(msg.header, risk, position))
        return array

    def _risk_sphere(self, header: Header, risk: ObstacleRisk, position) -> Marker:
        color = _THREAT_COLOR[risk.threat_level]
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'risks'
        marker.id = risk.track_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self._risk_diameter_m
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.3 + 0.5 * risk.risk_score
        marker.lifetime = self._lifetime.to_msg()
        return marker

    def _risk_label(self, header: Header, risk: ObstacleRisk, position) -> Marker:
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'risk_labels'
        marker.id = risk.track_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + self._risk_diameter_m / 2.0 + self._label_height_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._label_height_m
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'{_THREAT_NAME[risk.threat_level]} {risk.risk_score:.2f}'
        marker.lifetime = self._lifetime.to_msg()
        return marker

    def _publish_risks(self, array: MarkerArray) -> None:
        self._risks_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
