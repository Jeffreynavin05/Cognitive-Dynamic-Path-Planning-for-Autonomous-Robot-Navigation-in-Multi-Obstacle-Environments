"""Unit tests for Track's lifecycle state machine -- plain objects, no ROS node."""
import numpy as np
from interfaces.msg import TrackedObject

from cognitive_tracking.kalman_filter import KalmanFilter6D
from cognitive_tracking.track import Track


def _track(status=TrackedObject.STATUS_TENTATIVE, hits=1, misses=0):
    kf = KalmanFilter6D(
        initial_position=np.zeros(3), process_noise_std=0.5,
        measurement_noise_std=0.1, initial_velocity_variance=10.0)
    return Track(track_id=0, kf=kf, class_id=3, size=(0.4, 0.4, 0.4),
                 status=status, hits=hits, misses=misses)


def test_new_track_starts_tentative():
    track = _track()
    assert track.status == TrackedObject.STATUS_TENTATIVE
    assert track.hits == 1
    assert track.age == 1


def test_register_hit_confirms_after_threshold():
    track = _track(hits=2)  # one more hit reaches confirm_after_hits=3

    track.register_hit(class_id=3, size=(0.4, 0.4, 0.4), confirm_after_hits=3)

    assert track.status == TrackedObject.STATUS_CONFIRMED
    assert track.hits == 3
    assert track.misses == 0


def test_register_hit_does_not_confirm_before_threshold():
    track = _track(hits=1)

    track.register_hit(class_id=3, size=(0.4, 0.4, 0.4), confirm_after_hits=3)

    assert track.status == TrackedObject.STATUS_TENTATIVE
    assert track.hits == 2


def test_register_hit_recovers_occluded_track_to_confirmed():
    track = _track(status=TrackedObject.STATUS_OCCLUDED, hits=0, misses=2)

    track.register_hit(class_id=3, size=(0.4, 0.4, 0.4), confirm_after_hits=3)

    assert track.status == TrackedObject.STATUS_CONFIRMED


def test_register_hit_updates_class_and_size_from_newest_detection():
    track = _track()

    track.register_hit(class_id=2, size=(0.1, 0.2, 0.3), confirm_after_hits=3)

    assert track.class_id == 2
    assert track.size == (0.1, 0.2, 0.3)


def test_register_miss_marks_confirmed_track_occluded_at_threshold_one():
    track = _track(status=TrackedObject.STATUS_CONFIRMED, hits=3, misses=0)

    track.register_miss(occluded_after_misses=1, lost_after_misses=5)

    assert track.status == TrackedObject.STATUS_OCCLUDED
    assert track.hits == 0
    assert track.misses == 1


def test_register_miss_marks_lost_after_threshold():
    track = _track(status=TrackedObject.STATUS_OCCLUDED, misses=4)

    track.register_miss(occluded_after_misses=1, lost_after_misses=5)

    assert track.status == TrackedObject.STATUS_LOST


def test_register_miss_leaves_tentative_track_tentative_below_lost_threshold():
    track = _track(status=TrackedObject.STATUS_TENTATIVE, hits=1, misses=0)

    track.register_miss(occluded_after_misses=1, lost_after_misses=5)

    assert track.status == TrackedObject.STATUS_TENTATIVE


def test_age_increments_on_hit_and_miss():
    track = _track()

    track.register_hit(class_id=3, size=(0.4, 0.4, 0.4), confirm_after_hits=3)
    assert track.age == 2

    track.register_miss(occluded_after_misses=1, lost_after_misses=5)
    assert track.age == 3
