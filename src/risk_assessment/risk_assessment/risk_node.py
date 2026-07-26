"""Per-obstacle collision risk scoring for Phase 1.

Turns interfaces/PredictedTrajectoryArray (from motion_prediction, Module 5)
plus the robot's own /wheel/odom into interfaces/ObstacleRiskArray -- the
fixed contract dynamic_planner (Module 7) and visualization_node consume. Any
risk-model backend (this project's deterministic weighted-component scorer, a
learned model) must publish this exact shape so nothing downstream ever has to
change; see interfaces/msg/ObstacleRisk.msg's own header comment.

Staged the same way prediction_node/tracking_node are, so a future backend
swap only ever touches one stage:
    _odom_callback           -- INPUT (cache).  The only method touching the
                                 odometry subscriber; caches the robot's
                                 current position/velocity (world frame) for
                                 use by the next trajectories callback.
    _trajectories_callback   -- INPUT (drive).  Entry point; drives every
                                 stage below in order and is the only method
                                 touching the trajectories subscriber.
                                 Publishes synchronously once per incoming
                                 PredictedTrajectoryArray -- no separate
                                 timer, mirroring tracking_node/
                                 prediction_node's convention.
    _compute_risk             -- COMPUTE.       Per-trajectory risk scoring
                                 (risk_assessment.risk_model), swappable for a
                                 learned model later without touching any
                                 other stage.
    _build_risk_array         -- ASSEMBLE.      Pure message-building, no I/O
                                 -- directly unit-testable.
    _publish                  -- OUTPUT.        The only method that touches
                                 the publisher.

The robot's own future path is a locally-projected constant-velocity forecast
from cached odometry, not a real planned path -- dynamic_planner (Module 7)
does not exist yet. This is a documented Phase-1 stand-in (see
risk_model.py's module docstring and this package's README).
"""
import numpy as np
import rclpy
from interfaces.msg import ObstacleRisk, ObstacleRiskArray, PredictedTrajectory, PredictedTrajectoryArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from risk_assessment.risk_model import RiskParams, RiskResult, assess_obstacle_risk, rotate_vector_by_quaternion

ODOM_TOPIC = '/wheel/odom'
TRAJECTORIES_TOPIC = '/prediction/trajectories'
RISKS_TOPIC = '/risk/obstacle_risks'


class RiskNode(Node):

    def __init__(self):
        super().__init__('risk_node')

        self.declare_parameter('robot_radius_m', 0.2)
        self.declare_parameter('obstacle_radius_m', 0.4)
        self.declare_parameter('robot_position_std_m', 0.1)
        self.declare_parameter('weight_ttc', 0.35)
        self.declare_parameter('weight_prob', 0.35)
        self.declare_parameter('weight_speed', 0.15)
        self.declare_parameter('weight_distance', 0.15)
        self.declare_parameter('max_relative_speed_mps', 3.0)
        self.declare_parameter('max_distance_m', 5.0)
        self.declare_parameter('threat_medium_min', 0.25)
        self.declare_parameter('threat_high_min', 0.5)
        self.declare_parameter('threat_critical_min', 0.75)

        gp = self.get_parameter
        self._params = RiskParams(
            collision_radius_m=gp('robot_radius_m').value + gp('obstacle_radius_m').value,
            robot_position_std_m=gp('robot_position_std_m').value,
            weight_ttc=gp('weight_ttc').value,
            weight_prob=gp('weight_prob').value,
            weight_speed=gp('weight_speed').value,
            weight_distance=gp('weight_distance').value,
            max_relative_speed_mps=gp('max_relative_speed_mps').value,
            max_distance_m=gp('max_distance_m').value,
            threat_medium_min=gp('threat_medium_min').value,
            threat_high_min=gp('threat_high_min').value,
            threat_critical_min=gp('threat_critical_min').value,
        )

        # Cached from the most recent /wheel/odom message, world frame. No
        # risk can be assessed before the first odometry message arrives.
        self._robot_position = np.zeros(3)
        self._robot_velocity = np.zeros(3)
        self._have_odom = False

        self._odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self._odom_callback, 10)
        self._trajectories_sub = self.create_subscription(
            PredictedTrajectoryArray, TRAJECTORIES_TOPIC, self._trajectories_callback, 10)
        self._risks_pub = self.create_publisher(ObstacleRiskArray, RISKS_TOPIC, 10)

        self.get_logger().info(
            f'risk_node ready: subscribed to {TRAJECTORIES_TOPIC} and {ODOM_TOPIC}, publishing '
            f'ObstacleRiskArray on {RISKS_TOPIC} synchronously per prediction cycle '
            f'(collision_radius={self._params.collision_radius_m}m).')

    # ---- input: cache only, no processing --------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear = msg.twist.twist.linear

        self._robot_position = np.array([position.x, position.y, position.z])
        # nav_msgs/Odometry's twist is in the body (child_frame_id) frame;
        # rotate into the world frame to match /prediction/trajectories.
        velocity_body = np.array([linear.x, linear.y, linear.z])
        quaternion = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        self._robot_velocity = rotate_vector_by_quaternion(velocity_body, quaternion)
        self._have_odom = True

    # ---- input: drives every stage below, in order (Phase 2 backend swap) ------

    def _trajectories_callback(self, msg: PredictedTrajectoryArray) -> None:
        if not self._have_odom:
            # No robot state yet -- nothing to assess risk against.
            return

        risks = [self._compute_risk(msg.header, trajectory) for trajectory in msg.trajectories]

        array = self._build_risk_array(msg.header, risks)
        self._publish(array)

    # ---- compute (swappable independently of every other stage) ----------------

    def _compute_risk(self, header: Header, trajectory: PredictedTrajectory) -> ObstacleRisk:
        points = self._extract_points(header, trajectory)
        result = assess_obstacle_risk(
            trajectory.track_id, self._robot_position, self._robot_velocity, points, self._params)
        return self._to_obstacle_risk(result)

    def _extract_points(self, header: Header,
                         trajectory: PredictedTrajectory) -> list[tuple[float, np.ndarray, np.ndarray, np.ndarray]]:
        base_stamp = Time.from_msg(header.stamp)
        points = []
        for point in trajectory.points:
            offset_sec = (Time.from_msg(point.stamp) - base_stamp).nanoseconds / 1e9
            position = np.array([point.position.x, point.position.y, point.position.z])
            velocity = np.array([point.velocity.x, point.velocity.y, point.velocity.z])
            covariance = np.array(point.covariance)
            points.append((offset_sec, position, velocity, covariance))
        return points

    def _to_obstacle_risk(self, result: RiskResult) -> ObstacleRisk:
        risk = ObstacleRisk()
        risk.track_id = result.track_id
        risk.risk_score = result.risk_score
        risk.time_to_collision = result.time_to_collision
        risk.path_intersection_prob = result.path_intersection_prob
        risk.relative_speed = result.relative_speed
        risk.distance_to_robot = result.distance_to_robot
        risk.threat_level = result.threat_level
        return risk

    # ---- assemble (pure, no I/O -- directly unit-testable) ----------------------

    def _build_risk_array(self, header: Header, risks: list[ObstacleRisk]) -> ObstacleRiskArray:
        array = ObstacleRiskArray()
        array.header.stamp = header.stamp
        array.header.frame_id = header.frame_id
        array.risks = risks
        return array

    # ---- output -------------------------------------------------------------------

    def _publish(self, array: ObstacleRiskArray) -> None:
        self._risks_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RiskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
