# interfaces

Shared message contract for the cognitive navigation pipeline. Every other package
in this workspace depends on this one; this package depends on nothing project-specific.

## Why this package exists

Each stage of the pipeline (perception → tracking → prediction → risk → planner) is
meant to be independently replaceable — ground-truth detection swapped for YOLO,
the Kalman tracker swapped for DeepSORT, constant-velocity prediction swapped for an
LSTM — without touching any downstream node. That only works if the *shape* of the
data crossing each boundary is fixed and versioned in one place, decoupled from any
single implementation. Hence a standalone `interfaces` package with no node logic.

## Message pipeline

| Message | Published by | Consumed by |
|---|---|---|
| `DetectedObjectArray` | perception_node | tracking_node |
| `TrackedObjectArray` | tracking_node | prediction_node, visualization_node |
| `PredictedTrajectoryArray` | prediction_node | risk_node, visualization_node |
| `ObstacleRiskArray` | risk_node | planner_node, visualization_node |

Goal-sending reuses `nav2_msgs/action/NavigateToPose` directly rather than a custom
action — no reason to reinvent what Nav2 already provides.

## Design notes

- **Composed from standard types** (`geometry_msgs/Point`, `geometry_msgs/Vector3`,
  `std_msgs/Header`) wherever possible, so RViz, `tf2`, and other stock ROS2 tooling
  work on these messages without adapters.
- **`model_name` on `PredictedTrajectory`** identifies which predictor produced a
  forecast (`"constant_velocity"`, `"kalman_filter"`, `"lstm_v1"`, ...) purely for
  logging/debugging — the planner must never branch on it.
- **`ObstacleRisk` exposes its inputs** (TTC, path-intersection probability, relative
  speed, distance) alongside the final `risk_score`, not just the score, so risk
  decisions are explainable during evaluation/viva rather than a black box.
- **Covariances are flattened row-major arrays**, matching the convention used by
  `nav_msgs/Odometry` and `geometry_msgs/PoseWithCovariance`.

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces
source install/setup.bash
```

## Verify

```bash
ros2 interface list | grep interfaces
ros2 interface show interfaces/msg/ObstacleRisk
```
