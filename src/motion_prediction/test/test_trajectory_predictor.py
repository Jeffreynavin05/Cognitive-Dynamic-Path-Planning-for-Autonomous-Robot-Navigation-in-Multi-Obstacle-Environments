"""Unit tests for trajectory_predictor's constant-velocity forecast math. Pure
numpy, no ROS -- same testing convention as cognitive_tracking's
test_kalman_filter.py."""
import numpy as np

from motion_prediction.trajectory_predictor import predict_trajectory


def _flat_covariance(variance=1.0):
    return (np.eye(6) * variance).flatten()


def test_predict_trajectory_returns_one_point_per_step():
    points = predict_trajectory(
        position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0),
        covariance_flat=_flat_covariance(), process_noise_std=0.5,
        step_sec=0.1, num_steps=30)

    assert len(points) == 30


def test_offsets_are_multiples_of_step_sec():
    points = predict_trajectory(
        position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0),
        covariance_flat=_flat_covariance(), process_noise_std=0.5,
        step_sec=0.1, num_steps=5)

    offsets = [offset for offset, _, _, _ in points]
    assert np.allclose(offsets, [0.1, 0.2, 0.3, 0.4, 0.5])


def test_position_advances_linearly_by_constant_velocity():
    points = predict_trajectory(
        position=(0.0, 0.0, 0.0), velocity=(2.0, 1.0, 0.0),
        covariance_flat=_flat_covariance(), process_noise_std=0.5,
        step_sec=0.5, num_steps=3)

    # Constant-velocity model: position(t) = position(0) + velocity * t, exactly
    # (no acceleration term), regardless of how much process noise is added to
    # the covariance alongside it.
    _, last_position, _, _ = points[-1]
    assert np.allclose(last_position, [3.0, 1.5, 0.0])  # t = 1.5s


def test_velocity_is_unchanged_by_a_constant_velocity_forecast():
    points = predict_trajectory(
        position=(0.0, 0.0, 0.0), velocity=(2.0, -1.0, 0.5),
        covariance_flat=_flat_covariance(), process_noise_std=0.5,
        step_sec=0.1, num_steps=10)

    for _, _, velocity, _ in points:
        assert np.allclose(velocity, [2.0, -1.0, 0.5])


def test_position_covariance_grows_with_horizon():
    points = predict_trajectory(
        position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0),
        covariance_flat=_flat_covariance(), process_noise_std=0.5,
        step_sec=0.1, num_steps=10)

    traces = [np.trace(cov.reshape(3, 3)) for _, _, _, cov in points]
    assert all(later >= earlier for earlier, later in zip(traces, traces[1:]))
    assert traces[-1] > traces[0]


def test_position_covariance_is_row_major_length_nine():
    points = predict_trajectory(
        position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0),
        covariance_flat=_flat_covariance(), process_noise_std=0.5,
        step_sec=0.1, num_steps=1)

    _, _, _, covariance = points[0]
    assert covariance.shape == (9,)
