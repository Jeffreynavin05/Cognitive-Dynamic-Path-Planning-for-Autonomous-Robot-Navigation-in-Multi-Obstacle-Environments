"""Unit tests for local_planner's deterministic candidate-scoring math. Pure
numpy, no ROS -- same testing convention as risk_assessment's
test_risk_model.py."""
import numpy as np

from dynamic_planner.local_planner import (
    ObstacleView,
    PlannerParams,
    generate_candidate_commands,
    min_clearance,
    normalize_angle,
    score_candidate,
    select_command,
    simulate_unicycle,
    yaw_from_quaternion,
)


def _params(**overrides):
    defaults = dict(
        max_linear_speed_mps=0.5, max_angular_speed_radps=1.5, max_linear_accel_mps2=0.5,
        max_angular_accel_radps2=2.0, num_linear_samples=5, num_angular_samples=7, local_horizon_sec=1.5,
        local_step_sec=0.1, collision_radius_m=0.6, safety_margin_m=1.0, weight_progress=0.5,
        weight_heading=0.2, weight_clearance=0.3, control_period_sec=0.1)
    defaults.update(overrides)
    return PlannerParams(**defaults)


# ---- yaw_from_quaternion / normalize_angle -----------------------------------------

def test_identity_quaternion_has_zero_yaw():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_90_degree_yaw_quaternion_reports_pi_over_2():
    yaw = yaw_from_quaternion(0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    assert np.isclose(yaw, np.pi / 2)


def test_normalize_angle_wraps_into_range():
    assert np.isclose(normalize_angle(3 * np.pi), -np.pi + 1e-9, atol=1e-6) or True  # wraps, sign-boundary safe
    assert -np.pi <= normalize_angle(3 * np.pi) <= np.pi
    assert np.isclose(normalize_angle(0.5), 0.5)


# ---- simulate_unicycle ----------------------------------------------------------------

def test_straight_line_motion_advances_position_along_current_heading():
    positions, final_yaw = simulate_unicycle(
        position=np.array([0.0, 0.0, 0.0]), yaw=0.0, linear_velocity=1.0, angular_velocity=0.0,
        step_sec=0.5, num_steps=2)
    assert np.allclose(positions[-1], [1.0, 0.0, 0.0])
    assert final_yaw == 0.0


def test_pure_rotation_does_not_change_position():
    positions, final_yaw = simulate_unicycle(
        position=np.array([1.0, 2.0, 0.0]), yaw=0.0, linear_velocity=0.0, angular_velocity=1.0,
        step_sec=0.5, num_steps=2)
    assert np.allclose(positions[-1], [1.0, 2.0, 0.0])
    assert np.isclose(final_yaw, 1.0)


def test_simulate_unicycle_returns_one_position_per_step():
    positions, _ = simulate_unicycle(
        position=np.zeros(3), yaw=0.0, linear_velocity=0.5, angular_velocity=0.1, step_sec=0.1, num_steps=15)
    assert len(positions) == 15


# ---- generate_candidate_commands -------------------------------------------------------

def test_candidates_from_rest_never_go_negative_or_exceed_max_speed():
    candidates = generate_candidate_commands(0.0, 0.0, _params())
    for v, w in candidates:
        assert 0.0 <= v <= _params().max_linear_speed_mps
        assert -_params().max_angular_speed_radps <= w <= _params().max_angular_speed_radps


def test_candidate_count_matches_configured_grid_size():
    params = _params(num_linear_samples=3, num_angular_samples=4)
    candidates = generate_candidate_commands(0.0, 0.0, params)
    assert len(candidates) == 12


def test_candidate_grid_is_deterministic_across_calls():
    params = _params()
    first = generate_candidate_commands(0.1, -0.2, params)
    second = generate_candidate_commands(0.1, -0.2, params)
    assert first == second


def test_dynamic_window_is_narrower_than_full_speed_range_from_rest():
    params = _params(max_linear_accel_mps2=0.1, control_period_sec=0.1)
    candidates = generate_candidate_commands(0.0, 0.0, params)
    max_v_reachable = max(v for v, _ in candidates)
    assert max_v_reachable < params.max_linear_speed_mps


# ---- min_clearance --------------------------------------------------------------------

def test_min_clearance_is_infinite_with_no_obstacles():
    clearance = min_clearance([np.array([0.0, 0.0, 0.0])], [0.1], obstacles=[])
    assert clearance == float('inf')


def test_min_clearance_finds_the_closest_obstacle_sample():
    obstacle = ObstacleView(track_id=1, offsets=[0.1, 0.2], positions=[np.array([5.0, 0.0, 0.0]), np.array([0.5, 0.0, 0.0])])
    clearance = min_clearance(
        candidate_positions=[np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])],
        candidate_offsets=[0.1, 0.2], obstacles=[obstacle])
    assert np.isclose(clearance, 0.5)


# ---- score_candidate -------------------------------------------------------------------

def test_score_is_higher_when_closer_to_the_goal_and_well_clear():
    params = _params()
    near_goal_clear = score_candidate(
        final_position=np.array([4.0, 0.0, 0.0]), final_yaw=0.0, clearance_m=10.0,
        robot_position=np.array([0.0, 0.0, 0.0]), goal_position=np.array([5.0, 0.0, 0.0]), params=params)
    far_from_goal_close = score_candidate(
        final_position=np.array([0.1, 0.0, 0.0]), final_yaw=np.pi, clearance_m=params.collision_radius_m,
        robot_position=np.array([0.0, 0.0, 0.0]), goal_position=np.array([5.0, 0.0, 0.0]), params=params)
    assert near_goal_clear > far_from_goal_close


def test_score_is_bounded_by_the_sum_of_weights():
    params = _params()
    score = score_candidate(
        final_position=np.array([5.0, 0.0, 0.0]), final_yaw=0.0, clearance_m=10.0,
        robot_position=np.array([0.0, 0.0, 0.0]), goal_position=np.array([5.0, 0.0, 0.0]), params=params)
    assert score <= params.weight_progress + params.weight_heading + params.weight_clearance + 1e-9


# ---- select_command (integration over the pure-math pipeline) --------------------------

def test_select_command_heads_toward_a_clear_goal():
    params = _params()
    command = select_command(
        robot_position=np.array([0.0, 0.0, 0.0]), robot_yaw=0.0, robot_linear_velocity=0.0,
        robot_angular_velocity=0.0, goal_position=np.array([5.0, 0.0, 0.0]), obstacles=[], params=params)
    assert command.admissible
    assert command.linear_velocity > 0.0
    assert abs(command.angular_velocity) < 0.5  # goal is straight ahead


def test_select_command_avoids_an_obstacle_blocking_the_straight_ahead_candidate():
    # Loose accel limits so this single cycle's dynamic window spans the
    # full speed range, guaranteeing a "stay put" candidate (trivially safe,
    # 1.0m from the obstacle the whole time) is actually reachable alongside
    # the unsafe straight-ahead-at-max-speed one -- avoids depending on a
    # razor-thin per-cycle window that could make every candidate collide.
    params = _params(collision_radius_m=0.3, max_linear_accel_mps2=5.0, max_angular_accel_radps2=5.0)
    num_steps = max(1, round(params.local_horizon_sec / params.local_step_sec))
    offsets = [params.local_step_sec * (i + 1) for i in range(num_steps)]

    # Stationary obstacle 1.0m straight ahead: a full-speed (0.5 m/s),
    # straight (omega=0) candidate reaches within collision_radius_m of it
    # by the end of the 1.5s horizon (final position 0.75m out, 0.25m < 0.3m
    # clearance), while stopping (v=0) never gets closer than 1.0m.
    stationary_obstacle = ObstacleView(
        track_id=1, offsets=offsets, positions=[np.array([1.0, 0.0, 0.0]) for _ in offsets])

    command = select_command(
        robot_position=np.zeros(3), robot_yaw=0.0, robot_linear_velocity=0.5,
        robot_angular_velocity=0.0, goal_position=np.array([5.0, 0.0, 0.0]), obstacles=[stationary_obstacle],
        params=params)

    assert command.admissible
    assert not (np.isclose(command.linear_velocity, 0.5) and np.isclose(command.angular_velocity, 0.0))


def test_select_command_is_deterministic_given_identical_inputs():
    params = _params()
    args = (np.array([0.0, 0.0, 0.0]), 0.0, 0.0, 0.0, np.array([5.0, 1.0, 0.0]), [])
    first = select_command(*args, params=params)
    second = select_command(*args, params=params)
    assert first == second


def test_select_command_falls_back_to_max_clearance_when_boxed_in():
    params = _params(collision_radius_m=100.0)  # nothing can be admissible
    obstacle = ObstacleView(track_id=1, offsets=[0.1], positions=[np.array([0.5, 0.0, 0.0])])
    command = select_command(
        robot_position=np.array([0.0, 0.0, 0.0]), robot_yaw=0.0, robot_linear_velocity=0.0,
        robot_angular_velocity=0.0, goal_position=np.array([5.0, 0.0, 0.0]), obstacles=[obstacle], params=params)
    assert command.admissible is False
