"""Unit tests for KalmanFilter6D's predict/update math. Pure numpy, no ROS."""
import numpy as np

from cognitive_tracking.kalman_filter import KalmanFilter6D


def _kf(position=(0.0, 0.0, 0.0)):
    return KalmanFilter6D(
        initial_position=np.array(position),
        process_noise_std=0.5,
        measurement_noise_std=0.1,
        initial_velocity_variance=10.0)


def test_predict_advances_position_by_current_velocity_times_dt():
    kf = _kf()
    kf.x[3:6] = [1.0, 2.0, 0.0]  # give it a known velocity directly

    kf.predict(dt=2.0)

    assert np.allclose(kf.position, [2.0, 4.0, 0.0])
    # A constant-velocity predict step never changes velocity itself.
    assert np.allclose(kf.velocity, [1.0, 2.0, 0.0])


def test_predict_grows_covariance():
    kf = _kf()
    initial_trace = np.trace(kf.P)

    kf.predict(dt=1.0)

    assert np.trace(kf.P) > initial_trace


def test_update_pulls_state_toward_measurement():
    kf = _kf(position=(0.0, 0.0, 0.0))

    kf.update(np.array([1.0, 1.0, 1.0]))

    # Measurement noise is small relative to the prior, so the corrected position
    # should move toward the measurement without necessarily reaching it exactly.
    assert np.all(kf.position > 0.0)
    assert np.all(kf.position <= 1.0)


def test_update_shrinks_covariance():
    kf = _kf()
    initial_trace = np.trace(kf.P)

    kf.update(np.array([1.0, 1.0, 1.0]))

    assert np.trace(kf.P) < initial_trace


def test_covariance_flat_is_row_major_36_length():
    kf = _kf()

    flat = kf.covariance_flat

    assert flat.shape == (36,)
    assert np.allclose(flat.reshape(6, 6), kf.P)
