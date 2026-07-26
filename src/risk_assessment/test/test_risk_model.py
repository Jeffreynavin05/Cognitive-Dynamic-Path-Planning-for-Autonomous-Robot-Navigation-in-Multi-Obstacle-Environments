"""Unit tests for risk_model's collision-risk math. Pure numpy, no ROS -- same
testing convention as motion_prediction's test_trajectory_predictor.py."""
import numpy as np

from risk_assessment.risk_model import (
    RiskParams,
    assess_obstacle_risk,
    classify_threat,
    closing_speed,
    compute_risk_score,
    euclidean_distance,
    path_intersection_probability,
    project_robot_position,
    rotate_vector_by_quaternion,
    time_to_collision,
)


def _params(**overrides):
    defaults = dict(
        collision_radius_m=0.6, robot_position_std_m=0.1, weight_ttc=0.35, weight_prob=0.35,
        weight_speed=0.15, weight_distance=0.15, max_relative_speed_mps=3.0, max_distance_m=5.0,
        threat_medium_min=0.25, threat_high_min=0.5, threat_critical_min=0.75)
    defaults.update(overrides)
    return RiskParams(**defaults)


def _flat_covariance(variance=0.01):
    return np.eye(3).flatten() * variance


# ---- rotate_vector_by_quaternion -------------------------------------------------

def test_identity_quaternion_leaves_vector_unchanged():
    vector = np.array([1.0, 2.0, 3.0])
    rotated = rotate_vector_by_quaternion(vector, np.array([0.0, 0.0, 0.0, 1.0]))
    assert np.allclose(rotated, vector)


def test_90_degree_yaw_rotates_x_axis_velocity_into_y_axis():
    quaternion = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    rotated = rotate_vector_by_quaternion(np.array([1.0, 0.0, 0.0]), quaternion)
    assert np.allclose(rotated, [0.0, 1.0, 0.0], atol=1e-6)


# ---- project_robot_position -------------------------------------------------------

def test_project_robot_position_advances_linearly_by_constant_velocity():
    projected = project_robot_position(
        position=np.array([0.0, 0.0, 0.0]), velocity=np.array([2.0, 1.0, 0.0]), offset_sec=1.5)
    assert np.allclose(projected, [3.0, 1.5, 0.0])


# ---- time_to_collision --------------------------------------------------------------

def test_time_to_collision_is_negative_one_when_never_within_radius():
    ttc = time_to_collision(offsets=[0.1, 0.2, 0.3], distances=[5.0, 5.0, 5.0], collision_radius_m=0.6)
    assert ttc == -1.0


def test_time_to_collision_returns_first_offset_within_radius():
    ttc = time_to_collision(offsets=[0.1, 0.2, 0.3], distances=[2.0, 0.5, 0.1], collision_radius_m=0.6)
    assert ttc == 0.2


# ---- closing_speed -------------------------------------------------------------------

def test_closing_speed_is_positive_when_obstacle_approaches_directly():
    speed = closing_speed(
        robot_position=np.array([0.0, 0.0, 0.0]), robot_velocity=np.array([0.0, 0.0, 0.0]),
        obstacle_position=np.array([5.0, 0.0, 0.0]), obstacle_velocity=np.array([-1.0, 0.0, 0.0]))
    assert speed == 1.0


def test_closing_speed_is_negative_when_obstacle_moves_away():
    speed = closing_speed(
        robot_position=np.array([0.0, 0.0, 0.0]), robot_velocity=np.array([0.0, 0.0, 0.0]),
        obstacle_position=np.array([5.0, 0.0, 0.0]), obstacle_velocity=np.array([1.0, 0.0, 0.0]))
    assert speed == -1.0


def test_closing_speed_is_zero_when_robot_and_obstacle_coincide():
    speed = closing_speed(
        robot_position=np.array([1.0, 1.0, 0.0]), robot_velocity=np.array([0.0, 0.0, 0.0]),
        obstacle_position=np.array([1.0, 1.0, 0.0]), obstacle_velocity=np.array([1.0, 0.0, 0.0]))
    assert speed == 0.0


# ---- path_intersection_probability ---------------------------------------------------

def test_probability_is_near_one_at_zero_distance():
    prob = path_intersection_probability(0.0, _flat_covariance(), robot_position_std_m=0.1)
    assert prob == 1.0


def test_probability_decays_towards_zero_for_large_distance():
    prob = path_intersection_probability(20.0, _flat_covariance(), robot_position_std_m=0.1)
    assert prob < 0.01


def test_probability_is_lower_for_a_farther_closest_approach():
    near = path_intersection_probability(0.2, _flat_covariance(), robot_position_std_m=0.1)
    far = path_intersection_probability(1.0, _flat_covariance(), robot_position_std_m=0.1)
    assert far < near


# ---- compute_risk_score / classify_threat ---------------------------------------------

def test_risk_score_is_zero_when_every_component_is_at_its_floor():
    score = compute_risk_score(
        time_to_collision_sec=-1.0, horizon_sec=3.0, path_intersection_prob=0.0,
        relative_speed_mps=-1.0, distance_to_robot_m=10.0, params=_params())
    assert score == 0.0


def test_risk_score_is_high_for_imminent_close_fast_closing_obstacle():
    score = compute_risk_score(
        time_to_collision_sec=0.1, horizon_sec=3.0, path_intersection_prob=1.0,
        relative_speed_mps=3.0, distance_to_robot_m=0.1, params=_params())
    assert score > 0.9


def test_classify_threat_boundaries():
    params = _params()
    assert classify_threat(0.0, params) == 0
    assert classify_threat(0.25, params) == 1
    assert classify_threat(0.5, params) == 2
    assert classify_threat(0.75, params) == 3


# ---- assess_obstacle_risk (integration over the pure-math pipeline) -------------------

def _collision_course_points(num_steps=5, step_sec=0.1, start_x=1.0, speed=2.0, variance=0.01):
    points = []
    for i in range(1, num_steps + 1):
        offset = step_sec * i
        position = np.array([start_x - speed * offset, 0.0, 0.0])
        velocity = np.array([-speed, 0.0, 0.0])
        points.append((offset, position, velocity, _flat_covariance(variance)))
    return points


def _stationary_far_points(num_steps=5, step_sec=0.1, position=(20.0, 0.0, 0.0)):
    points = []
    for i in range(1, num_steps + 1):
        offset = step_sec * i
        points.append((offset, np.array(position, dtype=float), np.zeros(3), _flat_covariance()))
    return points


def test_assess_obstacle_risk_flags_a_direct_collision_course_as_higher_risk_than_a_distant_static_obstacle():
    params = _params()
    robot_position = np.zeros(3)
    robot_velocity = np.zeros(3)

    collision_result = assess_obstacle_risk(
        track_id=1, robot_position=robot_position, robot_velocity=robot_velocity,
        points=_collision_course_points(), params=params)
    distant_result = assess_obstacle_risk(
        track_id=2, robot_position=robot_position, robot_velocity=robot_velocity,
        points=_stationary_far_points(), params=params)

    assert collision_result.time_to_collision >= 0.0
    assert distant_result.time_to_collision == -1.0
    assert collision_result.risk_score > distant_result.risk_score
    assert collision_result.threat_level > distant_result.threat_level


def test_assess_obstacle_risk_preserves_track_id():
    result = assess_obstacle_risk(
        track_id=42, robot_position=np.zeros(3), robot_velocity=np.zeros(3),
        points=_stationary_far_points(), params=_params())
    assert result.track_id == 42


def test_assess_obstacle_risk_score_is_in_the_unit_interval():
    result = assess_obstacle_risk(
        track_id=1, robot_position=np.zeros(3), robot_velocity=np.zeros(3),
        points=_collision_course_points(), params=_params())
    assert 0.0 <= result.risk_score <= 1.0
    assert 0.0 <= result.path_intersection_prob <= 1.0
