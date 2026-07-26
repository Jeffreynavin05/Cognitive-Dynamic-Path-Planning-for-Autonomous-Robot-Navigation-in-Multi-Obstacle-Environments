"""Constant-velocity trajectory forecaster: pure math, no ROS/rclpy imports.

Reimplements the same constant-velocity state-transition (F) and process-noise
(Q) construction as cognitive_tracking.kalman_filter.KalmanFilter6D.predict(),
so a track's forecast covariance grows under the same model its Kalman filter
uses -- but the code lives here independently rather than importing
cognitive_tracking, per this workspace's rule that no pipeline-stage package
depends on another stage package for anything but its published topic (see
PROJECT_CONTEXT.md section 7 / 16). The two implementations may be
mathematically equivalent; they must stay architecturally decoupled.

Swappable independently of prediction_node.py: a future LSTM/transformer
backend replaces only predict_trajectory() (and whatever helpers it needs)
with the same (position, velocity, covariance) -> list-of-points shape.
"""
import numpy as np

STATE_DIM = 6
POSITION_DIM = 3


def _state_transition(dt: float) -> np.ndarray:
    """Constant-velocity F: position advances by velocity * dt, velocity unchanged."""
    F = np.eye(STATE_DIM)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F


def _process_noise(process_noise_std: float, dt: float) -> np.ndarray:
    """Isotropic white-noise-acceleration process model, scaled by dt -- same
    simplified form as KalmanFilter6D.predict()'s Q, not a fully coupled
    discretized Q."""
    return np.eye(STATE_DIM) * (process_noise_std ** 2) * dt


def propagate_state(state: np.ndarray, covariance: np.ndarray, dt: float,
                     process_noise_std: float) -> tuple[np.ndarray, np.ndarray]:
    """One constant-velocity forecast step forward by dt."""
    F = _state_transition(dt)
    Q = _process_noise(process_noise_std, dt)
    new_state = F @ state
    new_covariance = F @ covariance @ F.T + Q
    return new_state, new_covariance


def predict_trajectory(position: np.ndarray, velocity: np.ndarray, covariance_flat,
                        process_noise_std: float, step_sec: float,
                        num_steps: int) -> list[tuple[float, np.ndarray, np.ndarray, np.ndarray]]:
    """Forecast num_steps future states at step_sec spacing via repeated
    constant-velocity propagation, starting from a track's current
    (position, velocity, 6x6 covariance).

    Returns a list of (offset_sec, position(3,), velocity(3,),
    position_covariance_flat(9,)) tuples, one per forecast step -- the
    position covariance is the top-left 3x3 block of the propagated 6x6
    state covariance, flattened row-major to match TrajectoryPoint.msg.
    """
    state = np.concatenate([np.asarray(position, dtype=float), np.asarray(velocity, dtype=float)])
    covariance = np.asarray(covariance_flat, dtype=float).reshape(STATE_DIM, STATE_DIM)

    points = []
    offset = 0.0
    for _ in range(num_steps):
        state, covariance = propagate_state(state, covariance, step_sec, process_noise_std)
        offset += step_sec
        position_covariance = covariance[0:POSITION_DIM, 0:POSITION_DIM].flatten()
        points.append((offset, state[0:3].copy(), state[3:6].copy(), position_covariance))
    return points
