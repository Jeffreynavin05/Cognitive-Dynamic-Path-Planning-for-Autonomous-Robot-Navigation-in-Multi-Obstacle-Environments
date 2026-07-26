"""Unit tests for global_path's straight-line waypoint generation. Pure
numpy, no ROS -- same testing convention as motion_prediction's
test_trajectory_predictor.py."""
import numpy as np

from dynamic_planner.global_path import generate_straight_line_path


def test_path_starts_and_ends_at_the_given_points():
    path = generate_straight_line_path(
        start=np.array([0.0, 0.0, 0.0]), goal=np.array([5.0, 0.0, 0.0]), waypoint_spacing_m=1.0)
    assert np.allclose(path[0], [0.0, 0.0, 0.0])
    assert np.allclose(path[-1], [5.0, 0.0, 0.0])


def test_waypoints_are_no_farther_apart_than_the_configured_spacing():
    path = generate_straight_line_path(
        start=np.array([0.0, 0.0, 0.0]), goal=np.array([5.0, 0.0, 0.0]), waypoint_spacing_m=1.0)
    for a, b in zip(path, path[1:]):
        assert np.linalg.norm(b - a) <= 1.0 + 1e-9


def test_waypoints_are_all_colinear_with_start_and_goal():
    start = np.array([1.0, 1.0, 0.0])
    goal = np.array([4.0, 5.0, 0.0])
    path = generate_straight_line_path(start=start, goal=goal, waypoint_spacing_m=0.5)

    direction = (goal - start) / np.linalg.norm(goal - start)
    for waypoint in path:
        offset = waypoint - start
        projection_length = np.dot(offset, direction)
        projected_point = start + direction * projection_length
        assert np.allclose(waypoint, projected_point, atol=1e-9)


def test_coincident_start_and_goal_returns_a_two_point_path():
    point = np.array([2.0, 3.0, 0.0])
    path = generate_straight_line_path(start=point, goal=point, waypoint_spacing_m=1.0)
    assert len(path) == 2
    assert np.allclose(path[0], point)
    assert np.allclose(path[1], point)


def test_finer_spacing_produces_more_waypoints():
    start = np.array([0.0, 0.0, 0.0])
    goal = np.array([10.0, 0.0, 0.0])
    coarse = generate_straight_line_path(start=start, goal=goal, waypoint_spacing_m=5.0)
    fine = generate_straight_line_path(start=start, goal=goal, waypoint_spacing_m=0.5)
    assert len(fine) > len(coarse)
