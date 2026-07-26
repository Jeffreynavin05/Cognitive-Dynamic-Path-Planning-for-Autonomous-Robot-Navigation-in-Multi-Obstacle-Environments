"""Per-obstacle collision risk math: pure numpy, no ROS/rclpy imports.

Turns one interfaces/PredictedTrajectory (from motion_prediction, Module 5) plus
the robot's own current position/velocity into the explainable fields
interfaces/msg/ObstacleRisk.msg requires: time_to_collision,
path_intersection_prob, relative_speed, distance_to_robot, and the derived
risk_score/threat_level. Kept free of interfaces/rclpy imports, same convention
as motion_prediction.trajectory_predictor, so it is independently unit-testable
and so a future learned risk model can replace this module's internals without
risk_node.py, interfaces/ObstacleRiskArray, or /risk/obstacle_risks ever
changing.

Design notes (see PROJECT_CONTEXT.md section 15/16 for the full rationale):
    - The robot's own future path is a locally-projected constant-velocity
      forecast from its current odometry (project_robot_position), not a real
      planned path -- dynamic_planner (Module 7) does not exist yet. This is a
      documented Phase-1 stand-in, replaced once a real plan exists, without
      this module's public shape changing.
    - Every trajectory sample's timestamp offset is read directly from the
      TrajectoryPoint the caller passes in (risk_node derives it from
      point.stamp - header.stamp), not from a separately configured
      horizon_sec/step_sec -- risk_assessment never needs to know
      motion_prediction's sampling parameters to stay correct.
    - Obstacle radius is a fixed, configured constant (collision_radius_m),
      not derived from the track's real size, because PredictedTrajectory
      carries no size/class_id field -- adding one, or subscribing to
      /tracking/tracks directly, would break the documented single-input
      topic graph for this stage (PROJECT_CONTEXT.md section 5C).
    - path_intersection_prob is a closed-form Gaussian falloff on distance at
      closest approach, combining the obstacle's own propagated position
      covariance with a configured robot_position_std_m stand-in for
      localization uncertainty (Phase 1 has no real localization covariance
      to read yet). Deterministic, no sampling/RNG.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class RiskParams:
    collision_radius_m: float
    robot_position_std_m: float
    weight_ttc: float
    weight_prob: float
    weight_speed: float
    weight_distance: float
    max_relative_speed_mps: float
    max_distance_m: float
    threat_medium_min: float
    threat_high_min: float
    threat_critical_min: float


@dataclass
class RiskResult:
    track_id: int
    risk_score: float
    time_to_collision: float
    path_intersection_prob: float
    relative_speed: float
    distance_to_robot: float
    threat_level: int


def rotate_vector_by_quaternion(vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """Rotate a 3-vector by a quaternion (x, y, z, w). Used to express
    nav_msgs/Odometry's twist (published in the robot's body frame) in the
    same world frame as its pose and as /prediction/trajectories."""
    q_xyz = quaternion[:3]
    q_w = quaternion[3]
    t = 2.0 * np.cross(q_xyz, vector)
    return vector + q_w * t + np.cross(q_xyz, t)


def project_robot_position(position: np.ndarray, velocity: np.ndarray, offset_sec: float) -> np.ndarray:
    """Constant-velocity forward projection of the robot's own position --
    the Phase-1 stand-in for a real planned path (see module docstring)."""
    return position + velocity * offset_sec


def euclidean_distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def time_to_collision(offsets: list[float], distances: list[float], collision_radius_m: float) -> float:
    """First offset_sec at which the robot/obstacle distance drops to or
    below collision_radius_m; -1.0 if that never happens within the
    trajectory's horizon, matching ObstacleRisk.msg's own comment."""
    for offset, distance in zip(offsets, distances):
        if distance <= collision_radius_m:
            return float(offset)
    return -1.0


def closing_speed(robot_position: np.ndarray, robot_velocity: np.ndarray,
                   obstacle_position: np.ndarray, obstacle_velocity: np.ndarray) -> float:
    """Positive = obstacle closing on the robot, matching ObstacleRisk.msg's
    'closing speed toward the robot' comment."""
    line_of_sight = obstacle_position - robot_position
    distance = np.linalg.norm(line_of_sight)
    if distance < 1e-6:
        # Degenerate: robot and obstacle coincide, no direction to project onto.
        return 0.0
    unit_los = line_of_sight / distance
    relative_velocity = obstacle_velocity - robot_velocity
    return float(-np.dot(relative_velocity, unit_los))


def path_intersection_probability(min_distance: float, obstacle_covariance_flat: np.ndarray,
                                   robot_position_std_m: float) -> float:
    """Closed-form Gaussian falloff on the closest-approach distance, using
    the obstacle's own propagated position covariance (collapsed to an
    isotropic std) combined with a configured robot position uncertainty
    stand-in. 1.0 at zero distance, decaying towards 0.0 as min_distance grows
    past the combined uncertainty."""
    position_covariance = obstacle_covariance_flat.reshape(3, 3)
    obstacle_std = np.sqrt(max(np.trace(position_covariance) / 3.0, 0.0))
    combined_std = max(np.sqrt(obstacle_std ** 2 + robot_position_std_m ** 2), 1e-6)
    probability = np.exp(-0.5 * (min_distance / combined_std) ** 2)
    return float(np.clip(probability, 0.0, 1.0))


def _normalized_component(value: float, saturation: float, only_if_positive: bool = False) -> float:
    if saturation <= 0.0:
        return 0.0
    if only_if_positive and value <= 0.0:
        return 0.0
    return float(np.clip(value / saturation, 0.0, 1.0))


def compute_risk_score(time_to_collision_sec: float, horizon_sec: float, path_intersection_prob: float,
                        relative_speed_mps: float, distance_to_robot_m: float, params: RiskParams) -> float:
    """Weighted linear combination of normalized components -- deliberately
    linear and inspectable rather than a black-box function, per this
    workspace's 'clean architecture over sophistication' philosophy
    (PROJECT_CONTEXT.md section 10)."""
    ttc_component = 0.0 if time_to_collision_sec < 0.0 else _normalized_component(
        horizon_sec - time_to_collision_sec, horizon_sec)
    prob_component = float(np.clip(path_intersection_prob, 0.0, 1.0))
    speed_component = _normalized_component(
        relative_speed_mps, params.max_relative_speed_mps, only_if_positive=True)
    distance_component = _normalized_component(
        params.max_distance_m - distance_to_robot_m, params.max_distance_m)

    score = (params.weight_ttc * ttc_component + params.weight_prob * prob_component
             + params.weight_speed * speed_component + params.weight_distance * distance_component)
    return float(np.clip(score, 0.0, 1.0))


def classify_threat(risk_score: float, params: RiskParams) -> int:
    """Ordinal values match interfaces/msg/ObstacleRisk.msg's THREAT_* constants
    (LOW=0, MEDIUM=1, HIGH=2, CRITICAL=3) -- this module has no interfaces
    import, so risk_node assigns the returned int directly."""
    if risk_score >= params.threat_critical_min:
        return 3
    if risk_score >= params.threat_high_min:
        return 2
    if risk_score >= params.threat_medium_min:
        return 1
    return 0


def assess_obstacle_risk(track_id: int, robot_position: np.ndarray, robot_velocity: np.ndarray,
                          points: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]],
                          params: RiskParams) -> RiskResult:
    """Combine one obstacle's PredictedTrajectory (as a list of
    (offset_sec, position, velocity, position_covariance_flat) tuples -- the
    same per-step shape trajectory_predictor.predict_trajectory returns) with
    the robot's own projected trajectory into a single RiskResult.

    Assumes points is non-empty -- guaranteed by motion_prediction's own
    num_steps = max(1, ...) construction (see trajectory_predictor.py); every
    published PredictedTrajectory has at least one point.
    """
    offsets = [offset for offset, _, _, _ in points]
    obstacle_positions = [position for _, position, _, _ in points]
    obstacle_velocity = points[0][2]

    robot_positions = [project_robot_position(robot_position, robot_velocity, offset) for offset in offsets]
    distances = [euclidean_distance(r, o) for r, o in zip(robot_positions, obstacle_positions)]

    distance_to_robot = euclidean_distance(robot_position, obstacle_positions[0])
    relative_speed = closing_speed(robot_position, robot_velocity, obstacle_positions[0], obstacle_velocity)
    ttc = time_to_collision(offsets, distances, params.collision_radius_m)

    closest_index = int(np.argmin(distances))
    prob = path_intersection_probability(
        distances[closest_index], points[closest_index][3], params.robot_position_std_m)

    horizon_sec = offsets[-1]
    score = compute_risk_score(ttc, horizon_sec, prob, relative_speed, distance_to_robot, params)
    threat_level = classify_threat(score, params)

    return RiskResult(
        track_id=track_id, risk_score=score, time_to_collision=ttc, path_intersection_prob=prob,
        relative_speed=relative_speed, distance_to_robot=distance_to_robot, threat_level=threat_level)
