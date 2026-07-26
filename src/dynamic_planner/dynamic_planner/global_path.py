"""Straight-line global path generation: pure numpy, no ROS/rclpy imports.

Phase 1 has no map/SLAM/localization stack anywhere in this pipeline -- only
odometry (see PROJECT_CONTEXT.md section 5A/9b) -- so a real Nav2-style
global planner (which needs a map to search over) is not meaningfully
available. This module produces a straight line from the robot's current
position to the goal in the shared "world" frame as a documented Phase-1
stand-in; local_planner.py is what actually keeps the robot off obstacles by
locally deviating from it. Swappable independently of local_planner.py and
planner_node.py: a future real global planner (Nav2's, or one that consumes
an occupancy grid) replaces only generate_straight_line_path() with the same
(start, goal) -> list-of-waypoints shape.
"""
import numpy as np


def generate_straight_line_path(start: np.ndarray, goal: np.ndarray, waypoint_spacing_m: float) -> list[np.ndarray]:
    """Evenly-spaced waypoints from start to goal inclusive, spaced at most
    waypoint_spacing_m apart. Always returns at least [start, goal]."""
    distance = float(np.linalg.norm(goal - start))
    if distance < 1e-9:
        return [start.copy(), goal.copy()]

    num_segments = max(1, int(np.ceil(distance / waypoint_spacing_m)))
    return [start + (goal - start) * (i / num_segments) for i in range(num_segments + 1)]
