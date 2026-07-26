# risk_assessment

Module 6 of the cognitive navigation pipeline: turns `interfaces/PredictedTrajectoryArray`
(constant-velocity forecasts from `motion_prediction`) and the robot's own `/wheel/odom`
into `interfaces/ObstacleRiskArray` -- an explainable, per-obstacle collision-risk score --
the fixed contract `dynamic_planner` (Module 7) and `visualization_node` consume.

## Why this package exists

Per `interfaces/msg/ObstacleRisk.msg`'s own header comment and the top-level design note
in `interfaces/README.md`, a risk decision must expose its inputs -- time-to-collision,
path-intersection probability, relative speed, distance -- alongside the final `risk_score`,
not just the score. Keeping the risk *model* separate from the `ObstacleRiskArray`
*contract* is what lets this project's deterministic weighted-component scorer be replaced
by a learned model later without `dynamic_planner` (or anything downstream) changing at all.

## Node

| Node | Role |
|---|---|
| `risk_node` | Subscribes to `/prediction/trajectories` and `/wheel/odom`, publishes `ObstacleRiskArray` on `/risk/obstacle_risks` synchronously, once per incoming `PredictedTrajectoryArray` |

## Design notes

- **`risk_node` is staged into distinct methods**, mirroring `motion_prediction`/
  `cognitive_tracking`'s convention so a future backend swap only ever touches one stage:

  | Stage | Method | Job |
  |---|---|---|
  | Input (cache) | `_odom_callback` | Caches the robot's current position/velocity (world frame). The only method touching the odometry subscriber. |
  | Input (drive) | `_trajectories_callback` | Entry point; drives every stage below in order. The only method touching the trajectories subscriber. |
  | Compute | `_compute_risk` | Per-trajectory risk scoring (`risk_assessment.risk_model`). |
  | Assemble | `_build_risk_array` | Pure message-building, no I/O -- directly unit-testable. |
  | Output | `_publish` | The only method that touches the publisher. |

  The risk math (`risk_model.py`) is its own module with no ROS/rclpy imports,
  unit-testable with plain numpy arrays, independent of `risk_node`.

- **The robot's own future path is a locally-projected constant-velocity forecast from
  cached odometry, not a real planned path.** `dynamic_planner` (Module 7) does not exist
  yet, so there is no planned-path topic to consume. `risk_node` projects the robot's own
  trajectory the same way `motion_prediction` projects an obstacle's -- a documented Phase-1
  stand-in, replaced by Nav2's real global/local plan in Module 7 without `risk_node`'s
  published contract changing.

- **Obstacle radius is a fixed configured constant, not derived from real obstacle size.**
  `PredictedTrajectory` carries no size/class_id field (only position, velocity, and
  covariance), so `collision_radius_m` (`robot_radius_m` + `obstacle_radius_m`, both
  configured) is the same for every track. This keeps `risk_node`'s only meaningful input
  exactly as documented in `PROJECT_CONTEXT.md` section 5C (`/prediction/trajectories` only)
  -- subscribing to `/tracking/tracks` for real size/class_id was considered and rejected for
  Phase 1, to avoid depending on a non-adjacent stage's topic.

- **`path_intersection_prob` is a closed-form Gaussian falloff, not a sampled/Monte-Carlo
  estimate.** At the trajectory sample of closest approach, the obstacle's own propagated
  3x3 position covariance (already growing with horizon per `motion_prediction`) is
  collapsed to an isotropic std and combined with a configured `robot_position_std_m`
  stand-in for localization uncertainty (Phase 1 has no real localization covariance to
  read yet). `probability = exp(-0.5 * (min_distance / combined_std)^2)`. Deterministic, no
  RNG, no new dependency (no SciPy `erf` needed).

- **`risk_score` is a weighted linear combination of four normalized components** (TTC,
  path-intersection probability, closing speed, distance), weights and saturation points
  declared in `config/risk_params.yaml`. Deliberately linear and inspectable rather than a
  black-box function, consistent with this workspace's "clean architecture over algorithmic
  sophistication" philosophy (`PROJECT_CONTEXT.md` section 10).

- **Every trajectory sample's time offset is read directly from `TrajectoryPoint.stamp`**
  (`point.stamp - header.stamp`), not from a separately configured `horizon_sec`/`step_sec`.
  `risk_assessment` never needs to know `motion_prediction`'s sampling parameters to stay
  correct -- whatever horizon/spacing `prediction_node` actually published is what gets used.

- **Odometry's twist is rotated from the body frame into the world frame** before use,
  since `nav_msgs/Odometry.twist` is conventionally expressed in `child_frame_id` (the
  robot body), not the frame its `pose` is in -- `rotate_vector_by_quaternion` in
  `risk_model.py` handles this with the odometry message's own orientation quaternion.

- **Publishing is synchronous with trajectory arrival, not its own timer** -- same
  reasoning as `prediction_node`/`tracking_node`: `motion_prediction` already publishes at a
  fixed cadence, so a second independent timer here would add complexity without benefit in
  Phase 1. No risk is published until at least one `/wheel/odom` message has been received.

- **No build dependency on `motion_prediction` or `cognitive_tracking`.** This package only
  depends on `interfaces` (plus stock `nav_msgs`) for message types and consumes
  `/prediction/trajectories`/`/wheel/odom` purely as runtime topics, consistent with the
  dependency-decoupling convention established across every earlier stage.

## Parameters (`config/risk_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `robot_radius_m` | 0.2 | Approximate BeetleBot chassis radius, summed into `collision_radius_m` |
| `obstacle_radius_m` | 0.4 | Generic fixed obstacle radius (no per-track size is available -- see above) |
| `robot_position_std_m` | 0.1 | Stand-in for real robot localization uncertainty, used in `path_intersection_prob` |
| `weight_ttc` / `weight_prob` / `weight_speed` / `weight_distance` | 0.35 / 0.35 / 0.15 / 0.15 | `risk_score` component weights, intended to sum to 1.0 |
| `max_relative_speed_mps` | 3.0 | Closing speed at/above which the speed component of `risk_score` saturates |
| `max_distance_m` | 5.0 | Distance at/beyond which the distance component of `risk_score` is zero |
| `threat_medium_min` / `threat_high_min` / `threat_critical_min` | 0.25 / 0.5 / 0.75 | `risk_score` cut points for `threat_level` |

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces motion_prediction risk_assessment
source install/setup.bash
```

## Run

```bash
# terminal 1
ros2 launch simulation world.launch.py

# terminal 2, once Gazebo and the obstacles have spawned
ros2 launch cognitive_perception perception.launch.py

# terminal 3
ros2 launch cognitive_tracking tracking.launch.py

# terminal 4
ros2 launch motion_prediction prediction.launch.py

# terminal 5
ros2 launch risk_assessment risk.launch.py
```

## Test

**Automated (pure logic, no Gazebo/ROS graph needed):**
```bash
colcon test --packages-select risk_assessment
colcon test-result --verbose
```

**Manual (confirm risks flow end-to-end):**
```bash
ros2 topic hz /risk/obstacle_risks
ros2 topic echo /risk/obstacle_risks --once
```
You should see one `ObstacleRisk` per currently forecast track, each with `risk_score`/
`path_intersection_prob` in `[0, 1]`, `threat_level` in `{0,1,2,3}`, and `time_to_collision`
either `-1.0` or a value within the incoming trajectory's horizon.

## Goal

This module prioritizes clean, explainable, deterministic architecture over risk-model
sophistication -- it's the reference implementation a future learned risk model should be
able to replace (by rewriting only `risk_model.py`) without `interfaces/ObstacleRiskArray`,
`/risk/obstacle_risks`, or any downstream node ever changing.
