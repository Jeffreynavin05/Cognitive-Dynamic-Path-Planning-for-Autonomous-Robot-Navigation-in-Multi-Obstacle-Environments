"""Detection<->track data association.

Greedy nearest-neighbor over a Euclidean distance cost matrix for Phase 1. Kept as
free functions operating on plain numpy arrays (not on Track/DetectedObject types)
so a Hungarian-algorithm replacement later only needs a new function with the same
(cost_matrix, gating_threshold) -> (matches, unmatched_tracks, unmatched_detections)
signature -- tracking_node's _associate() is the only call site that would need to
change which function it calls.
"""
import numpy as np


def build_cost_matrix(track_positions: np.ndarray, detection_positions: np.ndarray) -> np.ndarray:
    """(num_tracks, 3) x (num_detections, 3) -> (num_tracks, num_detections) Euclidean
    distance matrix. Shaped (N, 0) / (0, M) / (0, 0) safely when either side is empty,
    since numpy broadcasting alone won't do the right thing for a truly empty axis."""
    num_tracks = len(track_positions)
    num_detections = len(detection_positions)
    if num_tracks == 0 or num_detections == 0:
        return np.zeros((num_tracks, num_detections))
    diff = track_positions[:, np.newaxis, :] - detection_positions[np.newaxis, :, :]
    return np.linalg.norm(diff, axis=2)


def greedy_nearest_neighbor(cost_matrix: np.ndarray, gating_threshold: float):
    """Match tracks to detections by repeatedly picking the globally closest
    still-unmatched pair, until nothing left is within gating_threshold. Global
    (not row-by-row) so processing order can't starve a detection that's actually
    closest to a track considered later.

    Returns (matches, unmatched_track_idx, unmatched_detection_idx): matches is a
    list of (track_idx, detection_idx) index pairs into the caller's own track and
    detection lists (not track IDs).
    """
    num_tracks, num_detections = cost_matrix.shape
    unmatched_tracks = set(range(num_tracks))
    unmatched_detections = set(range(num_detections))
    matches: list[tuple[int, int]] = []

    if num_tracks == 0 or num_detections == 0:
        return matches, sorted(unmatched_tracks), sorted(unmatched_detections)

    candidates = [
        (cost_matrix[t, d], t, d)
        for t in range(num_tracks)
        for d in range(num_detections)
        if cost_matrix[t, d] <= gating_threshold
    ]
    candidates.sort(key=lambda candidate: candidate[0])

    for _distance, track_idx, detection_idx in candidates:
        if track_idx in unmatched_tracks and detection_idx in unmatched_detections:
            matches.append((track_idx, detection_idx))
            unmatched_tracks.discard(track_idx)
            unmatched_detections.discard(detection_idx)

    return matches, sorted(unmatched_tracks), sorted(unmatched_detections)
