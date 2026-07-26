"""Deterministic, risk-aware local velocity-command planner: pure numpy, no
ROS/rclpy imports.

A small, fixed, deterministically-ordered grid of (linear, angular) velocity
candidates is generated within the current dynamic window (accel-limited
reachable range this control cycle -- see generate_candidate_commands()),
each is forward-simulated over a short horizon with simple unicycle
kinematics, and scored by a weighted linear combination of goal progress,
heading alignment, and obstacle clearance -- the same "expose the inputs,
combine them linearly, pick the best deterministically" philosophy as
risk_assessment.risk_model (PROJECT_CONTEXT.md section 10/16). No random
sampling anywhere, unlike a true MPPI controller: the same inputs always
produce the same command.

Kept free of interfaces/rclpy imports, same convention as
motion_prediction.trajectory_predictor and risk_assessment.risk_model, so it
is independently unit-testable and a future learned/MPPI backend can replace
this module's internals without planner_node.py, /cmd_vel_nav, or the
NavigateToPose action ever changing.

Obstacle geometry (ObstacleView) is expected to come from
interfaces/PredictedTrajectoryArray, joined by track_id with
interfaces/ObstacleRiskArray by planner_node -- this module only needs
geometry, never risk_score/threat_level directly. ObstacleRiskArray's
threat_level/time_to_collision are used by planner_node as a separate,
coarser emergency-stop safety gate (see planner_node.py), not folded into
this module's per-candidate scoring, per PROJECT_CONTEXT.md's approved
"clean division of labour" design decision for Module 7.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class PlannerParams:
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_accel_mps2: float
    max_angular_accel_radps2: float
    num_linear_samples: int
    num_angular_samples: int
    local_horizon_sec: float
    local_step_sec: float
    collision_radius_m: float
    safety_margin_m: float
    weight_progress: float
    weight_heading: float
    weight_clearance: float
    control_period_sec: float


@dataclass
class ObstacleView:
    """One obstacle's forecast geometry, time-aligned samples matching a
    PredictedTrajectory's own points -- no risk fields, see module docstring."""
    track_id: int
    offsets: list[float]
    positions: list[np.ndarray]


@dataclass
class PlannedCommand:
    linear_velocity: float
    angular_velocity: float
    admissible: bool  # False only when every candidate this cycle stayed within
    # collision_radius_m of an obstacle ("boxed in") and this is the
    # least-bad (max-clearance) fallback rather than a goal-scored choice.


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) from a quaternion, for a planar ground robot."""
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def simulate_unicycle(position: np.ndarray, yaw: float, linear_velocity: float, angular_velocity: float,
                       step_sec: float, num_steps: int) -> tuple[list[np.ndarray], float]:
    """Forward-simulate a constant (v, omega) unicycle command. Returns the
    per-step position list and the final yaw."""
    x, y, z = position
    current_yaw = yaw
    positions = []
    for _ in range(num_steps):
        x += linear_velocity * np.cos(current_yaw) * step_sec
        y += linear_velocity * np.sin(current_yaw) * step_sec
        current_yaw = normalize_angle(current_yaw + angular_velocity * step_sec)
        positions.append(np.array([x, y, z]))
    return positions, current_yaw


def generate_candidate_commands(current_linear_velocity: float, current_angular_velocity: float,
                                 params: PlannerParams) -> list[tuple[float, float]]:
    """Fixed, deterministically-ordered grid of (v, omega) candidates within
    this cycle's dynamic window -- the range reachable from the current
    command given the configured acceleration limits and control period."""
    v_lo = max(0.0, current_linear_velocity - params.max_linear_accel_mps2 * params.control_period_sec)
    v_hi = min(params.max_linear_speed_mps, current_linear_velocity
               + params.max_linear_accel_mps2 * params.control_period_sec)
    v_lo, v_hi = min(v_lo, v_hi), max(v_lo, v_hi)

    w_lo = max(-params.max_angular_speed_radps, current_angular_velocity
               - params.max_angular_accel_radps2 * params.control_period_sec)
    w_hi = min(params.max_angular_speed_radps, current_angular_velocity
               + params.max_angular_accel_radps2 * params.control_period_sec)
    w_lo, w_hi = min(w_lo, w_hi), max(w_lo, w_hi)

    linear_samples = np.linspace(v_lo, v_hi, params.num_linear_samples)
    angular_samples = np.linspace(w_lo, w_hi, params.num_angular_samples)
    return [(float(v), float(w)) for v in linear_samples for w in angular_samples]


def _nearest_sample(obstacle: ObstacleView, offset_sec: float) -> np.ndarray:
    """The obstacle's forecast position at the sample closest to offset_sec
    -- obstacle trajectories and candidate simulations aren't guaranteed to
    share identical step spacing, so this avoids requiring an exact match."""
    differences = [abs(offset - offset_sec) for offset in obstacle.offsets]
    return obstacle.positions[int(np.argmin(differences))]


def min_clearance(candidate_positions: list[np.ndarray], candidate_offsets: list[float],
                   obstacles: list[ObstacleView]) -> float:
    """Smallest robot-to-obstacle distance across every simulated step and
    every obstacle -- infinity if there are no obstacles to check."""
    if not obstacles:
        return float('inf')

    minimum = float('inf')
    for offset, position in zip(candidate_offsets, candidate_positions):
        for obstacle in obstacles:
            distance = float(np.linalg.norm(position - _nearest_sample(obstacle, offset)))
            minimum = min(minimum, distance)
    return minimum


def _clamp01(value: float) -> float:
    return float(np.clip(value, 0.0, 1.0))


def score_candidate(final_position: np.ndarray, final_yaw: float, clearance_m: float, robot_position: np.ndarray,
                     goal_position: np.ndarray, params: PlannerParams) -> float:
    """Weighted linear combination of goal progress, heading alignment, and
    obstacle clearance -- deliberately linear and inspectable, matching
    risk_assessment.risk_model.compute_risk_score's philosophy."""
    distance_now = float(np.linalg.norm(goal_position - robot_position))
    distance_at_end = float(np.linalg.norm(goal_position - final_position))
    max_possible_progress = params.max_linear_speed_mps * params.local_horizon_sec
    progress_component = (_clamp01((distance_now - distance_at_end) / max_possible_progress)
                           if max_possible_progress > 0.0 else 0.0)

    bearing_to_goal = float(np.arctan2(goal_position[1] - final_position[1], goal_position[0] - final_position[0]))
    heading_error = abs(normalize_angle(bearing_to_goal - final_yaw))
    heading_component = _clamp01(1.0 - heading_error / np.pi)

    clearance_span = max(params.safety_margin_m - params.collision_radius_m, 1e-6)
    clearance_component = _clamp01((clearance_m - params.collision_radius_m) / clearance_span)

    return (params.weight_progress * progress_component + params.weight_heading * heading_component
            + params.weight_clearance * clearance_component)


def select_command(robot_position: np.ndarray, robot_yaw: float, robot_linear_velocity: float,
                    robot_angular_velocity: float, goal_position: np.ndarray, obstacles: list[ObstacleView],
                    params: PlannerParams) -> PlannedCommand:
    """Score every candidate in this cycle's dynamic window and return the
    best one, deterministically. Candidates that stay clear of every
    obstacle by more than collision_radius_m over the whole horizon are
    'admissible' and scored on progress/heading/clearance; if none are
    admissible (the robot is boxed in), the candidate with the greatest
    minimum clearance is returned instead, flagged via
    PlannedCommand.admissible=False."""
    num_steps = max(1, round(params.local_horizon_sec / params.local_step_sec))
    offsets = [params.local_step_sec * (i + 1) for i in range(num_steps)]

    simulated = []
    for linear_velocity, angular_velocity in generate_candidate_commands(
            robot_linear_velocity, robot_angular_velocity, params):
        positions, final_yaw = simulate_unicycle(
            robot_position, robot_yaw, linear_velocity, angular_velocity, params.local_step_sec, num_steps)
        clearance = min_clearance(positions, offsets, obstacles)
        simulated.append((linear_velocity, angular_velocity, clearance, positions[-1], final_yaw))

    admissible = [candidate for candidate in simulated if candidate[2] > params.collision_radius_m]
    pool = admissible if admissible else simulated

    best_command = None
    best_score = None
    for linear_velocity, angular_velocity, clearance, final_position, final_yaw in pool:
        score = (score_candidate(final_position, final_yaw, clearance, robot_position, goal_position, params)
                 if admissible else clearance)
        if best_score is None or score > best_score:
            best_score = score
            best_command = (linear_velocity, angular_velocity)

    return PlannedCommand(
        linear_velocity=best_command[0], angular_velocity=best_command[1], admissible=bool(admissible))
