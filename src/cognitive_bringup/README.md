# cognitive_bringup

The unnumbered integration package (`PROJECT_CONTEXT.md` section 3/15): one-command startup
of the complete Phase-1 pipeline, RViz configuration, and a debugging `visualization_node`.
Introduces no new algorithms and makes no changes to any of the eight numbered pipeline
packages — its entire job is bringing up what they already built.

## Why this package exists

Every numbered module (`cognitive_perception` through `robot_controller`) is complete and
independently launchable, but nothing before this package ties them into a single command or
gives them a shared view. `cognitive_bringup` is that seam: it depends on every other package
(the only package in this workspace allowed to, since integrating them *is* its job — see
"Design notes" below), and nothing depends on it.

## Nodes

| Node | Role |
|---|---|
| `visualization_node` | Independently subscribes to `/tracking/tracks`, `/prediction/trajectories`, `/risk/obstacle_risks` and publishes a `visualization_msgs/MarkerArray` per stage for RViz. Not part of the control path. |

## Launch files

| File | Role |
|---|---|
| `bringup_sim.launch.py` | **Phase-1 entry point.** Gazebo (`simulation/launch/world.launch.py`) + a `world`→`odom` static transform + `pipeline.launch.py` (`use_sim_time:=true`, `output_topic:=/cmd_vel`). |
| `bringup_hardware.launch.py` | **Phase-2 entry point (stub).** `pipeline.launch.py` only (`use_sim_time:=false`, `output_topic:=/cmd_vel_gate`) — no BeetleBot sensor-driver bringup, since that's existing platform infrastructure this project doesn't build (see "Design notes"). |
| `pipeline.launch.py` | Shared by both of the above. Includes every stage launch file (`perception` → `controller`) plus `visualization_node` and (optionally) RViz. Never included directly by a user. |

## Design notes

- **Two separate top-level launch files, not one file with a `sim:=true/false` branch.**
  `bringup_sim.launch.py` and `bringup_hardware.launch.py` share `pipeline.launch.py` but are
  otherwise textually independent — a launch-time boolean mistake can't accidentally start
  Gazebo on the real BeetleBot or vice versa. Approved design decision.

- **The `world`→`odom` static transform.** Every pipeline message's `header.frame_id` is the
  hardcoded constant `"world"` (`cognitive_perception`'s `detection_frame_id`,
  `PROJECT_CONTEXT.md` section 8), but no node anywhere in this workspace ever publishes a TF
  frame actually named `"world"` — the only TF chain that exists is
  `odom → base_footprint → ...`, from the diff-drive plugin bridged in
  `simulation/launch/world.launch.py`. Without a link between the two, RViz has two
  disconnected trees and cannot render the robot model and any pipeline marker/path together.
  `bringup_sim.launch.py` publishes that one missing link — a fixed `world`→`odom` transform
  equal to the robot's spawn pose, read directly from `simulation/config/simulation_params.yaml`
  at launch time (not hand-duplicated: same "read the canonical file" pattern
  `world.launch.py` itself already uses for `robot_spawn_x`/`_y`/`_yaw`). Exact at `t=0` by
  construction; only as accurate afterward as the diff-drive plugin's own odometry integration
  — the same category of Phase-1 approximation already documented for `risk_node`'s
  self-projected path (section 9b/14/17). Visualization-only: no pipeline node subscribes to
  or depends on this transform, and it does not modify any existing package. Approved design
  decision.

- **Three marker topics, not one.** `visualization_node` publishes `/visualization/tracks_markers`,
  `/visualization/trajectories_markers`, and `/visualization/risk_markers` independently rather
  than combining everything into a single `MarkerArray`, so each pipeline stage can be toggled
  on/off separately in RViz's Displays panel. Mirrors this node's own already-documented
  "independent multi-topic subscription" pattern (section 6/9c/15). Approved design decision.

- **Risk markers are positioned via a `track_id` join, not a position field on `ObstacleRisk`.**
  `interfaces/msg/ObstacleRisk.msg` carries no position of its own (its own header comment:
  "not a raw sensor measurement"), so `visualization_node` caches the latest
  `PredictedTrajectory` per `track_id` and reads a risk marker's position from that trajectory's
  first point — the same by-`track_id` join `dynamic_planner`'s `planner_node._join_obstacles`
  already uses to combine geometry with risk priority (section 9c/16), extended to a second
  consumer without any `interfaces` message change. A risk with no matching cached trajectory
  that cycle is silently skipped, precedented by `_join_obstacles` doing the same (section 14).

- **`bringup_hardware.launch.py` is a documented stub, not a complete real-robot bringup.** It
  proves the pipeline-side launch argument seam (`use_sim_time`, `output_topic`) already works
  for Phase 2 without any code change, but does not launch the BeetleBot's own sensor drivers
  (RPLidar C1, Pi Camera V1.3, LSM6DSRTR IMU) or the `/cmd_vel_gate` arbitration node — that is
  existing BeetleBot-platform infrastructure this project assumes exists but does not build,
  the same assumption already documented for `/cmd_vel_gate` itself (section 9d/17).

- **No modifications to any numbered pipeline package.** Nothing in `cognitive_bringup`
  required a bug fix elsewhere; every existing package's launch file, topic name, and message
  contract is used exactly as documented.

## Parameters (`config/visualization_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `track_marker_min_size_m` | 0.2 | Floor applied to a track's marker scale so a `(0,0,0)` size never renders invisibly |
| `track_label_height_m` | 0.3 | Height of each `id`/status text label above its track/risk marker |
| `trajectory_line_width_m` | 0.05 | Line width of each trajectory's `LINE_STRIP` marker |
| `trajectory_uncertainty_max_radius_m` | 2.0 | Clamp on the covariance-derived uncertainty sphere drawn at a trajectory's final point |
| `risk_marker_diameter_m` | 0.6 | Diameter of each risk sphere marker |
| `marker_lifetime_sec` | 1.0 | How long RViz keeps a marker before expiring it client-side if no replacement arrives |

## Build

```bash
cd cognitive_navigation_ws
colcon build
source install/setup.bash
```

## Run

**Phase 1 (Gazebo), one command:**
```bash
ros2 launch cognitive_bringup bringup_sim.launch.py
```
This starts Gazebo, spawns the BeetleBot and every obstacle, publishes the `world`→`odom`
transform, and launches all seven pipeline nodes plus `visualization_node` and RViz (loaded
with `rviz/bringup.rviz`). Send a goal once the world has finished spawning:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}}}}"
```

Skip RViz (e.g. headless):
```bash
ros2 launch cognitive_bringup bringup_sim.launch.py use_rviz:=false
```

**Phase 2 (real BeetleBot) stub:**
```bash
ros2 launch cognitive_bringup bringup_hardware.launch.py
```
Assumes the BeetleBot's own sensor-driver bringup (`/scan`, `/imu/data`, `/pi_camera/image_raw`,
`/wheel/odom`, `/cmd_vel_gate`) is already running — see "Design notes" above.

Every package's own per-node launch file (`ros2 launch cognitive_perception perception.launch.py`,
etc.) still works standalone exactly as before — `cognitive_bringup` only adds a shared entry
point, it does not replace them.

## Test

**Automated (pure logic, no Gazebo/ROS graph needed):**
```bash
colcon test --packages-select cognitive_bringup
colcon test-result --verbose
```

**Manual (confirm RViz actually shows the pipeline):**
```bash
ros2 launch cognitive_bringup bringup_sim.launch.py
```
After sending a goal, confirm in RViz: the robot model moves under `TF`/`RobotModel`, obstacle
spheres appear under `TrackMarkers` colored by track status, forecast lines appear under
`TrajectoryMarkers`, and colored risk spheres appear under `RiskMarkers` near obstacles on a
collision course, all in the same `world`-fixed-frame view as the robot and `/scan`.

## Goal

The closing module of this workspace's Phase-1 reference implementation: everything from
`camera_node`/`lidar_node` through `controller_node` was built to be independently correct and
independently testable; this package is what proves it, showing the whole thing running and
visible with a single command.
