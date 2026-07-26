# dynamic_planner

Module 7 of the cognitive navigation pipeline: hosts a `nav2_msgs/action/NavigateToPose`
action server and turns `interfaces/ObstacleRiskArray`, `interfaces/PredictedTrajectoryArray`,
and the robot's own `/wheel/odom` into velocity commands on `/cmd_vel_nav` -- the fixed
contract `robot_controller` (Module 8) consumes.

## Why this package exists

Per `PROJECT_CONTEXT.md` section 1's contract-first thesis, this package owns everything
between an explainable per-obstacle risk assessment and an actual velocity command. Keeping
the planner *implementation* separate from the `/cmd_vel_nav`/`NavigateToPose` *contract* is
what lets this project's deterministic local planner be replaced by a real Nav2/MPPI stack
later without `robot_controller` (or anything downstream) changing at all.

## Node

| Node | Role |
|---|---|
| `planner_node` | Hosts a `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) action server; subscribes to `/risk/obstacle_risks`, `/prediction/trajectories`, `/wheel/odom`; publishes `Twist` on `/cmd_vel_nav` at a fixed control rate once a goal is active, and `nav_msgs/Path` on `/planner/global_path` once per accepted goal |

## Design notes

- **No real Nav2 bringup required.** `planner_node` hosts the `NavigateToPose` action server
  itself -- no `bt_navigator`, no lifecycle-managed costmap/planner/controller servers, no C++
  `nav2_costmap_2d` plugin. This workspace is pure `ament_python`; a real Nav2 costmap layer is
  a C++ pluginlib plugin, which would have been this workspace's first C++ node-bearing package
  and a heavy bringup dependency just for Phase 1. Approved design decision -- see
  `PROJECT_CONTEXT.md` section 16. `/cmd_vel_nav` and the `NavigateToPose` action are the fixed
  published interfaces a real Nav2 stack can replace this package's internals behind, later,
  without either changing.

- **Deterministic weighted-scoring local planner, not MPPI.** `local_planner.py` generates a
  small, fixed, deterministically-ordered grid of `(v, omega)` candidates (never randomly
  sampled), forward-simulates each with simple unicycle kinematics, and scores every
  *admissible* one (clear of every obstacle by more than `collision_radius_m` across the whole
  horizon) with a weighted linear combination of goal progress, heading alignment, and
  clearance -- the same explainable, inspectable philosophy as
  `risk_assessment.risk_model.compute_risk_score`. The same inputs always produce the same
  command. Approved design decision, given "deterministic behavior" is an explicit Module 7
  priority and MPPI is inherently stochastic.

- **Obstacle geometry comes from `/prediction/trajectories`, not `/risk/obstacle_risks`.**
  `ObstacleRisk.msg` carries no position field -- it is explicitly "not a raw sensor
  measurement" per its own header comment. `planner_node._join_obstacles` combines cached
  `PredictedTrajectoryArray` (geometry: position per forecast offset) with cached
  `ObstacleRiskArray` (explainable priority), joined by `track_id`. `local_planner.py` itself
  only ever sees geometry (`ObstacleView`), never `risk_score`/`threat_level` -- those are used
  exclusively as a coarse **emergency-stop safety gate**
  (`planner_node._emergency_stop_triggered`): any `THREAT_CRITICAL` track with
  `time_to_collision` at or below `emergency_stop_ttc_sec` forces an immediate stop regardless
  of candidate scoring. Approved design decision -- precedented by `visualization_node`'s own
  documented independent multi-topic subscription pattern (`PROJECT_CONTEXT.md` sections 6/15).
  No `interfaces` message was changed.

- **Straight-line global path, not a real Nav2 global planner.** Phase 1 has no map/SLAM/
  localization stack anywhere in this pipeline -- only odometry -- so a real global planner
  (which needs a map to search over) isn't meaningfully available yet. `global_path.py`
  generates a straight line from the robot's current position to the goal, published once per
  accepted goal on `/planner/global_path` (`nav_msgs/Path`) purely for explainability/RViz;
  `local_planner.py`'s candidate scoring targets the goal position directly (heading/progress
  are computed against the goal, not path waypoints), since a straight line's heading is
  identical everywhere along it -- an explicit lookahead/pure-pursuit mechanism would add no
  behavioral value here. Same "documented Phase-1 stand-in" spirit as `risk_node`'s odometry
  self-projection.

- **Timer-driven control loop, not synchronous-per-message.** Every prior node (Modules 4-6)
  publishes synchronously once per incoming message, with no independent timer -- a
  deliberately repeated convention (`PROJECT_CONTEXT.md` section 16). `planner_node` breaks it:
  a velocity-command publisher cannot go silent between upstream messages the way a
  pass-through stage can, so `_control_loop` runs on its own fixed-rate timer
  (`control_rate_hz`, default 10 Hz), always recomputing from the latest cached inputs.
  Approved design decision.

- **`MultiThreadedExecutor`, not `rclpy.spin()`.** A direct consequence of the control-loop
  decision above: the action server's long-running `_execute_callback` (which blocks in a
  poll loop until the goal succeeds or is cancelled) and the fixed-rate `_control_loop` timer
  must run concurrently, which a single-threaded executor cannot do without deadlocking.
  `main()` uses `MultiThreadedExecutor` with the action server and the
  timer/subscriptions on separate `MutuallyExclusiveCallbackGroup`s. This is the first node in
  this workspace to deviate from the plain `rclpy.init -> rclpy.spin -> destroy_node/shutdown`
  pattern (`PROJECT_CONTEXT.md` section 11).

- **One active goal at a time.** `_goal_callback` rejects a new goal while one is already
  executing, rather than preempting it -- the simplest correct behavior for a Phase-1
  reference implementation; a real Nav2 stack's preemption semantics can replace this later.

- **Staged design**, mirroring every prior module's convention, extended for the
  action-server + timer shape:

  | Stage | Method | Job |
  |---|---|---|
  | Input (cache) | `_odom_callback` | Robot position/yaw/body-frame velocity |
  | Input (cache) | `_risks_callback` | Latest `ObstacleRiskArray`, by `track_id` |
  | Input (cache) | `_trajectories_callback` | Latest `PredictedTrajectoryArray`, by `track_id` |
  | Input (goal) | `_goal_callback` / `_cancel_callback` / `_execute_callback` | Standard `NavigateToPose` action-server handlers |
  | Control (drive) | `_control_loop` | Timer callback; entry point for every stage below, once per cycle while a goal is active |
  | Join | `_join_obstacles` | Combines cached trajectories (geometry) with cached risks (priority), by `track_id` |
  | Plan | `_compute_command` | `dynamic_planner.local_planner.select_command()` |
  | Assemble | `_build_cmd_vel` / `_build_global_path` | Pure message-building, no I/O |
  | Output | `_publish_cmd` / `_publish_global_path` | The only methods that touch their respective publishers |

  `local_planner.py` and `global_path.py` are their own modules with no ROS/rclpy imports,
  unit-testable with plain numpy arrays, independent of `planner_node`.

- **No build dependency on `risk_assessment` or `motion_prediction`.** This package only
  depends on `interfaces` (plus stock `nav2_msgs`/`nav_msgs`/`geometry_msgs`) for message
  types and consumes `/risk/obstacle_risks`/`/prediction/trajectories`/`/wheel/odom` purely as
  runtime topics, consistent with the dependency-decoupling convention established across
  every earlier stage.

## Parameters (`config/planner_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `control_rate_hz` | 10.0 | Fixed-rate control loop frequency |
| `max_linear_speed_mps` / `max_angular_speed_radps` | 0.5 / 1.5 | Kinematic limits (approximate BeetleBot values) |
| `max_linear_accel_mps2` / `max_angular_accel_radps2` | 0.5 / 2.0 | Acceleration limits defining each cycle's dynamic window |
| `num_linear_samples` / `num_angular_samples` | 5 / 7 | Candidate `(v, omega)` grid density |
| `local_horizon_sec` / `local_step_sec` | 1.5 / 0.1 | Per-candidate forward-simulation horizon/spacing |
| `robot_radius_m` / `obstacle_radius_m` | 0.2 / 0.4 | Summed into `collision_radius_m`; independently declared from `risk_assessment`'s parameters of the same name |
| `safety_margin_m` | 1.0 | Distance beyond `collision_radius_m` at which the clearance score component saturates |
| `weight_progress` / `weight_heading` / `weight_clearance` | 0.5 / 0.2 / 0.3 | `local_planner` score component weights |
| `goal_tolerance_m` | 0.15 | Distance to goal at/below which `NavigateToPose` succeeds |
| `emergency_stop_ttc_sec` | 0.5 | `THREAT_CRITICAL` + TTC at/below this forces an immediate stop |
| `waypoint_spacing_m` | 0.5 | Spacing for the visualization-only `/planner/global_path` |

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces risk_assessment motion_prediction dynamic_planner
source install/setup.bash
```

## Run

```bash
# terminal 1
ros2 launch simulation world.launch.py

# terminals 2-5, once Gazebo and the obstacles have spawned
ros2 launch cognitive_perception perception.launch.py
ros2 launch cognitive_tracking tracking.launch.py
ros2 launch motion_prediction prediction.launch.py
ros2 launch risk_assessment risk.launch.py

# terminal 6
ros2 launch dynamic_planner planner.launch.py

# terminal 7, send a goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}}}}"
```

## Test

**Automated (pure logic, no Gazebo/ROS graph needed):**
```bash
colcon test --packages-select dynamic_planner
colcon test-result --verbose
```

**Manual (confirm the full loop flows end-to-end):**
```bash
ros2 topic echo /planner/global_path --once
ros2 topic hz /cmd_vel_nav
```
After sending a goal, you should see one `nav_msgs/Path` on `/planner/global_path` (start at
the robot's position, end at the goal), then `/cmd_vel_nav` publishing at `control_rate_hz`
until the robot is within `goal_tolerance_m`, at which point the action reports success and
`/cmd_vel_nav` stops.

## Goal

This module prioritizes clean, explainable, deterministic architecture over planning
sophistication -- it's the reference implementation a future real Nav2/MPPI stack should be
able to replace (by rewriting `local_planner.py`, and eventually delegating the action server
itself to Nav2's `bt_navigator`) without `/cmd_vel_nav`, the `NavigateToPose` action, or any
downstream node ever changing.
