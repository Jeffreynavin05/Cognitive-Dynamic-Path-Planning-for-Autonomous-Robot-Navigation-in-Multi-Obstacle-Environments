"""Track: one object's identity, Kalman estimate, and lifecycle bookkeeping.

Lifecycle transitions live here (not in tracking_node) so they're unit-testable
against a plain Track instance with no ROS node/publisher involved --
tracking_node's Lifecycle Management stage just calls register_hit()/
register_miss() once per track per cycle.
"""
from dataclasses import dataclass

from interfaces.msg import TrackedObject

from cognitive_tracking.kalman_filter import KalmanFilter6D


@dataclass
class Track:
    track_id: int
    kf: KalmanFilter6D
    class_id: int
    size: tuple[float, float, float]
    status: int = TrackedObject.STATUS_TENTATIVE
    hits: int = 1
    misses: int = 0
    age: int = 1

    def register_hit(self, class_id: int, size: tuple[float, float, float],
                      confirm_after_hits: int) -> None:
        """Matched a detection this cycle. class_id/size are taken from the newest
        detection outright -- no voting/fusion in Phase 1 (see this package's
        README)."""
        self.class_id = class_id
        self.size = size
        self.hits += 1
        self.misses = 0
        self.age += 1
        if self.status == TrackedObject.STATUS_TENTATIVE and self.hits >= confirm_after_hits:
            self.status = TrackedObject.STATUS_CONFIRMED
        elif self.status == TrackedObject.STATUS_OCCLUDED:
            self.status = TrackedObject.STATUS_CONFIRMED

    def register_miss(self, occluded_after_misses: int, lost_after_misses: int) -> None:
        """No detection matched this cycle. Position/velocity were already coasted
        forward by KalmanFilter6D.predict() before this is called."""
        self.hits = 0
        self.misses += 1
        self.age += 1
        if self.misses >= lost_after_misses:
            self.status = TrackedObject.STATUS_LOST
        elif self.misses >= occluded_after_misses and self.status in (
                TrackedObject.STATUS_CONFIRMED, TrackedObject.STATUS_OCCLUDED):
            self.status = TrackedObject.STATUS_OCCLUDED
