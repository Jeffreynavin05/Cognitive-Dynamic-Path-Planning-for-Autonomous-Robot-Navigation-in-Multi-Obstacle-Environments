"""Unit tests for greedy nearest-neighbor association. Pure numpy in/out, no ROS."""
import numpy as np

from cognitive_tracking.association import build_cost_matrix, greedy_nearest_neighbor


def test_build_cost_matrix_euclidean_distances():
    tracks = np.array([[0.0, 0.0, 0.0]])
    detections = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 1.0]])

    matrix = build_cost_matrix(tracks, detections)

    assert matrix.shape == (1, 2)
    assert np.isclose(matrix[0, 0], 5.0)
    assert np.isclose(matrix[0, 1], 1.0)


def test_build_cost_matrix_handles_empty_sides():
    assert build_cost_matrix(np.zeros((0, 3)), np.array([[0.0, 0.0, 0.0]])).shape == (0, 1)
    assert build_cost_matrix(np.array([[0.0, 0.0, 0.0]]), np.zeros((0, 3))).shape == (1, 0)
    assert build_cost_matrix(np.zeros((0, 3)), np.zeros((0, 3))).shape == (0, 0)


def test_greedy_matches_closest_pair_first_globally():
    # Track 0 is close to detection 1, track 1 is close to detection 0 -- this
    # confirms the match is found by global distance, not row processing order.
    cost_matrix = np.array([
        [5.0, 0.1],
        [0.2, 5.0],
    ])

    matches, unmatched_tracks, unmatched_detections = greedy_nearest_neighbor(
        cost_matrix, gating_threshold=1.0)

    assert set(matches) == {(0, 1), (1, 0)}
    assert unmatched_tracks == []
    assert unmatched_detections == []


def test_greedy_respects_gating_threshold():
    cost_matrix = np.array([[2.0]])

    matches, unmatched_tracks, unmatched_detections = greedy_nearest_neighbor(
        cost_matrix, gating_threshold=1.0)

    assert matches == []
    assert unmatched_tracks == [0]
    assert unmatched_detections == [0]


def test_greedy_one_track_does_not_double_match():
    cost_matrix = np.array([[0.1, 0.2]])  # one track, two nearby detections

    matches, unmatched_tracks, unmatched_detections = greedy_nearest_neighbor(
        cost_matrix, gating_threshold=1.0)

    assert matches == [(0, 0)]
    assert unmatched_tracks == []
    assert unmatched_detections == [1]


def test_greedy_handles_empty_inputs():
    matches, unmatched_tracks, unmatched_detections = greedy_nearest_neighbor(
        np.zeros((0, 0)), gating_threshold=1.0)

    assert (matches, unmatched_tracks, unmatched_detections) == ([], [], [])
