"""Constant-velocity Kalman filter over a single track's state [x, y, z, vx, vy, vz].

Pure math, no ROS/rclpy imports, so it's unit-testable with plain numpy arrays and
swappable independently of Track or tracking_node (e.g. for a constant-acceleration
model later) without touching association or lifecycle logic.
"""
import numpy as np

STATE_DIM = 6
MEASUREMENT_DIM = 3


class KalmanFilter6D:
    """One track's motion estimator. predict() and update() are kept separate
    (rather than a single step()) so tracking_node can predict every existing
    track once per cycle regardless of association outcome, then update only the
    ones that matched a detection -- see tracking_node's staged Predict/Update
    split."""

    def __init__(self, initial_position: np.ndarray, process_noise_std: float,
                 measurement_noise_std: float, initial_velocity_variance: float):
        self.x = np.zeros(STATE_DIM)
        self.x[0:3] = initial_position

        # Position starts exact (from the detection that spawned this track);
        # velocity is unknown at spawn, so it starts with deliberately high
        # uncertainty rather than an assumed 0.
        self.P = np.eye(STATE_DIM)
        self.P[3:, 3:] *= initial_velocity_variance

        self._process_noise_std = process_noise_std

        self.H = np.zeros((MEASUREMENT_DIM, STATE_DIM))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0
        self.R = np.eye(MEASUREMENT_DIM) * (measurement_noise_std ** 2)

    def predict(self, dt: float) -> None:
        """Advance state/covariance by dt under a constant-velocity model."""
        F = np.eye(STATE_DIM)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Simple isotropic white-noise-acceleration process model, scaled by dt --
        # not a fully coupled discretized Q, but sufficient for Phase 1's
        # constant-velocity assumption where clean architecture matters more than
        # tracking accuracy. Revisit if a future backend needs a tighter model.
        Q = np.eye(STATE_DIM) * (self._process_noise_std ** 2) * dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, measurement: np.ndarray) -> None:
        """Correct the prediction with an associated detection's position."""
        innovation = measurement - self.H @ self.x
        innovation_cov = self.H @ self.P @ self.H.T + self.R
        kalman_gain = self.P @ self.H.T @ np.linalg.inv(innovation_cov)

        self.x = self.x + kalman_gain @ innovation
        self.P = (np.eye(STATE_DIM) - kalman_gain @ self.H) @ self.P

    @property
    def position(self) -> np.ndarray:
        return self.x[0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:6]

    @property
    def covariance_flat(self) -> np.ndarray:
        """Row-major flattened 6x6, matching TrackedObject.covariance's documented
        convention (see interfaces/README.md's design notes)."""
        return self.P.flatten()
