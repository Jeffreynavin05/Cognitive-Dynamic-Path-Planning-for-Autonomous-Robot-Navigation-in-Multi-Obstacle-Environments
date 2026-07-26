# PROJECT_CONTEXT.md

Canonical engineering reference for `cognitive_navigation_ws`. This document is the
single source of truth for the project's architecture, conventions, and status.
Read this file before inspecting the repository or making changes — package
READMEs cover implementation detail for their own package; this file covers
everything that spans package boundaries.

Keep this document current: whenever a module's status changes, a new
architectural decision is made, or a convention changes, update the relevant
section here in the same change.

## Contents

1. [Project overview](#1-project-overview)
2. [Overall ROS2 architecture](#2-overall-ros2-architecture)
3. [Package responsibilities](#3-package-responsibilities)
4. [Current implementation status](#4-current-implementation-status)
5. [Complete topic graph](#5-complete-topic-graph)
6. [Message flow](#6-message-flow)
7. [Interfaces overview](#7-interfaces-overview)
8. [Module 3 implementation details (cognitive_perception)](#8-module-3-implementation-details-cognitive_perception)
9. [Module 4 implementation details (cognitive_tracking)](#9-module-4-implementation-details-cognitive_tracking)
9a. [Module 5 implementation details (motion_prediction)](#9a-module-5-implementation-details-motion_prediction)
9b. [Module 6 implementation details (risk_assessment)](#9b-module-6-implementation-details-risk_assessment)
9c. [Module 7 implementation details (dynamic_planner)](#9c-module-7-implementation-details-dynamic_planner)
9d. [Module 8 implementation details (robot_controller)](#9d-module-8-implementation-details-robot_controller)
10. [Design philosophy](#10-design-philosophy)
11. [Coding conventions](#11-coding-conventions)
12. [Testing conventions](#12-testing-conventions)
13. [Configuration conventions](#13-configuration-conventions)
14. [Known limitations](#14-known-limitations)
15. [Planned integration work](#15-planned-integration-work)
16. [Important architectural decisions made during implementation](#16-important-architectural-decisions-made-during-implementation)
17. [Assumptions and rationale behind each design choice](#17-assumptions-and-rationale-behind-each-design-choice)

---

## 1. Project overview

**Cognitive Dynamic Path Planning for Autonomous Robot Navigation in
Multi-Obstacle Environments** is a final-year B.Tech project implemented as a
ROS2 Jazzy workspace (`cognitive_navigation_ws`) targeting Ubuntu 24.04.

The project has two phases:

- **Phase 1 (current):** built and validated entirely in Gazebo Harmonic, using
  a hand-authored multi-obstacle arena with a mix of static and randomly-moving
  dynamic obstacles. Perception in this phase reads Gazebo ground truth directly
  rather than running a real detector.
- **Phase 2 (future):** the same pipeline, unmodified at the package-interface
  level, deployed onto the physical **BeetleBot** platform — a Raspberry Pi 5
  (Ubuntu 24.04, ROS2 Jazzy) with an RPLidar C1, Pi Camera V1.3, LSM6DSRTR IMU,
  and a 4WD differential-style drivetrain via an STM32 "Lyra" motor controller.

Two engineering theses underpin every design decision in this workspace:

1. **Contract-first pipeline decoupling.** Every stage of the pipeline
   (perception → tracking → prediction → risk → planning) is independently
   replaceable because the *shape* of the data crossing each stage boundary is
   fixed and versioned in one place (the `interfaces` package), decoupled from
   any single stage's implementation. Ground-truth detection can become YOLO,
   the Kalman tracker can become DeepSORT or ByteTrack, the constant-velocity
   predictor can become an LSTM — none of that requires touching a neighboring
   stage, because no stage package depends on another stage package for
   anything but its published topic.
2. **Digital-twin parity.** The `simulation` package exposes the exact same
   topic names and message types as the real BeetleBot's documented hardware
   interface (`/scan`, `/imu/data`, `/pi_camera/image_raw`, `/wheel/odom`).
   Every node from perception onward is written to be unaware of whether it is
   talking to Gazebo or the physical robot.

## 2. Overall ROS2 architecture

Pipeline (identical in intent to the top-level `README.md`, expanded with
current implementation status):

```
camera_node ─┐
lidar_node ──┴─→ perception_node → tracking_node → prediction_node → risk_node → planner_node → controller_node
              (Module 3, done)   (Module 4, done)  (Module 5, done)  (Module 6, done) (Module 7, done) (Module 8, done)
                                          │               │              │            │
                                          └───────────────┴──────────────┴────────────┴──→ visualization_node (planned)
```

Two packages sit outside this left-to-right chain and support it:

- `interfaces` — no nodes, only message definitions. Every stage package depends
  on it; it depends on nothing project-specific.
- `simulation` — Phase-1-only. Provides the Gazebo world, the BeetleBot URDF,
  and the randomized obstacle spawner/wander controller. No pipeline-stage
  package depends on it at build time; it is a runtime topic source only.

The workspace follows the standard `colcon`/`ament` layout: `src/` holds every
package's source, `build/`, `install/`, and `log/` are generated artifacts
(git-ignored). All node-bearing packages are `ament_python`; `interfaces` is
`ament_cmake` because message generation requires `rosidl_generate_interfaces`.

## 3. Package responsibilities

| Package | Build type | Status | Responsibility |
|---|---|---|---|
| `interfaces` | ament_cmake | done | Shared, versioned message contract for every pipeline stage boundary |
| `simulation` | ament_python | done | Gazebo Harmonic world, BeetleBot URDF, randomized static/dynamic obstacle spawner and wander controller ("digital twin" boundary) |
| `cognitive_perception` | ament_python | done (Module 3) | `camera_node`, `lidar_node`, `perception_node` — ground-truth (Phase 1) / real-sensor (Phase 2) object detection, publishing `DetectedObjectArray` |
| `cognitive_tracking` | ament_python | done (Module 4) | `tracking_node` — Kalman-filter multi-object tracking with persistent IDs, publishing `TrackedObjectArray` |
| `motion_prediction` | ament_python | done (Module 5) | `prediction_node` — constant-velocity trajectory forecasting per `CONFIRMED`/`OCCLUDED` tracked object, publishing `PredictedTrajectoryArray` |
| `risk_assessment` | ament_python | done (Module 6) | `risk_node` — per-obstacle collision risk scoring (TTC, path intersection, relative speed, distance) against the robot's own odometry-projected path, publishing `ObstacleRiskArray` |
| `dynamic_planner` | ament_python | done (Module 7) | `planner_node` — self-hosted `NavigateToPose` action server; deterministic, risk-aware local planner (no real Nav2 bringup/costmap plugin), publishing velocity commands on `/cmd_vel_nav` |
| `robot_controller` | ament_python | done (Module 8) | `controller_node` — relays `/cmd_vel_nav` onto a deployment-selected output topic (`/cmd_vel` sim / `/cmd_vel_gate` hardware), with a command-staleness watchdog safety stop |
| `cognitive_bringup` | ament_python (planned) | planned (integration, unnumbered) | Top-level launch files, RViz configs, world files, `visualization_node` |

No pipeline-stage package (`cognitive_perception`, `cognitive_tracking`, and
onward) lists another pipeline-stage package as a build dependency in its
`package.xml`. Each depends only on `interfaces` for message types; anything
else it consumes from an upstream stage is a runtime-only topic subscription.
`simulation` is never a dependency of any pipeline-stage package.

## 4. Current implementation status

| Package | Status | Automated tests | Verified |
|---|---|---|---|
| `interfaces` | done | — (message-only package; no logic to unit test) | Builds cleanly under `colcon build` |
| `simulation` | done | `test/test_geometry.py` (geometry/spatial-sampling logic) | Manually verified in Gazebo Harmonic (obstacle spawn, wander, wall avoidance) |
| `cognitive_perception` | done | `test_perception_node.py` (6 tests), `test_sensor_placeholders.py` (2 tests) | Manually verified against live Gazebo ground truth |
| `cognitive_tracking` | done | `test_kalman_filter.py` (5), `test_association.py` (6), `test_track.py` (9), `test_tracking_node.py` (7) — 27 tests total | Built with `colcon build`; unit suite green (`colcon test-result`: 36 tests workspace-wide, 0 failures); additionally smoke-tested live — `tracking_node` run standalone, fed fabricated `DetectedObjectArray` messages via `ros2 topic pub`, `/tracking/tracks` echoed and confirmed correct `track_id` stability, `STATUS_TENTATIVE → STATUS_CONFIRMED` transition at hit 3, growing `age`, and shrinking/coupling covariance across predict/update cycles |
| `motion_prediction` | done (Module 5) | `test_trajectory_predictor.py` (6), `test_prediction_node.py` (9) — 15 tests total | Built with `colcon build`; unit suite green (`colcon test-result`: 15 tests, 0 failures); additionally smoke-tested live — `prediction_node` run standalone, fed a fabricated `TrackedObjectArray` via `ros2 topic pub`, `/prediction/trajectories` echoed and confirmed `model_name: constant_velocity`, 30 `TrajectoryPoint`s at the default 3.0s/0.1s horizon/step, and growing position covariance across the array |
| `risk_assessment` | done (Module 6) | `test_risk_model.py` (17), `test_risk_node.py` (8) — 25 tests total | Built with `colcon build`; unit suite green (`colcon test-result`: 76 tests workspace-wide, 0 failures); additionally smoke-tested live — `risk_node` run standalone, fed a fabricated `Odometry` and `PredictedTrajectoryArray` via `ros2 topic pub`, `/risk/obstacle_risks` echoed and confirmed correct `distance_to_robot`, `relative_speed`, `time_to_collision`, `path_intersection_prob`, `risk_score`, and `threat_level` for a direct collision-course obstacle |
| `dynamic_planner` | done (Module 7) | `test_global_path.py` (5), `test_local_planner.py` (18), `test_planner_node.py` (13) — 36 tests total | Built with `colcon build` (all 7 workspace packages); unit suite green (`colcon test-result`: 117 tests workspace-wide, 0 failures beyond the pre-existing `interfaces` xmllint issue predating this module); additionally smoke-tested live in three scenarios — `planner_node` run standalone, fed a fabricated `Odometry` via `ros2 topic pub` and a goal via `ros2 action send_goal`: (1) far, obstacle-free goal — `/planner/global_path` published once (correct start/end waypoints) and `/cmd_vel_nav` published the expected dynamic-window-limited forward command (`linear.x=0.05`, matching hand-derived expectations) at 10 Hz; (2) a `THREAT_CRITICAL` risk with `time_to_collision=0.1` forced `/cmd_vel_nav` to all-zero throughout, despite a far, otherwise-clear goal; (3) a goal already within `goal_tolerance_m` made the action report `SUCCEEDED` immediately |
| `robot_controller` | done (Module 8) | `test_controller_node.py` (8 tests) | Built with `colcon build` (all 8 workspace packages); unit suite green (`colcon test-result`: 125 tests workspace-wide, 0 failures beyond the pre-existing `interfaces` xmllint issue predating this module); additionally smoke-tested live in three scenarios — `controller_node` run standalone, fed fabricated `Twist` messages via `ros2 topic pub`: (1) relay correctness — each `/cmd_vel_nav` message reappeared immediately on `/cmd_vel`; (2) watchdog — real silence exceeding `cmd_timeout_sec` (including the natural startup latency between separate `ros2 topic pub` invocations) reliably produced a zero `Twist` on `/cmd_vel`, with relay resuming correctly on the next fresh command; (3) `output_topic:=/cmd_vel_gate` override confirmed via `ros2 node info` to redirect the publisher, proving the Phase-2 deployment path |
| `cognitive_bringup` | planned | — | Not started |

## 5. Complete topic graph

### A. Hardware/simulation bridge topics
Bridged by `simulation/launch/world.launch.py`'s `ros_gz_bridge` `parameter_bridge`
node. Names and types match the real BeetleBot's documented interface (digital
twin principle, §1):

| Topic | Type | Direction | Real-robot equivalent |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | RPLidar C1 |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo → ROS | LSM6DSRTR IMU |
| `/pi_camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | Pi Camera V1.3 |
| `/wheel/odom` | `nav_msgs/Odometry` | Gazebo → ROS | Real wheel encoder odometry |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS | Standard TF tree |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS | Standard joint state |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | N/A (sim-time only) |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | Sim's raw diff-drive input, published by `controller_node` (`robot_controller`, Module 8) in Phase 1. **Not** the real robot's Nav2 output topic — see `/cmd_vel_nav` below |

`risk_node` (`risk_assessment`, Module 6) additionally subscribes to `/wheel/odom` directly — the
only pipeline-stage node so far to consume a category-A bridge topic instead of only a
pipeline-contract topic, since there is no real planned-path topic to consume until
`dynamic_planner` (Module 7) exists (see §9b, §16).

### B. Ground-truth-only topics (Phase 1 simulation, no real-robot equivalent)

| Topic | Type | Published by |
|---|---|---|
| `/model/static_obstacle_<i>/pose_static` (i = 0..`num_static_obstacles`-1, default 4) | `tf2_msgs/TFMessage` | Per-obstacle `PosePublisher` plugin, bridged per-model |
| `/model/dynamic_obstacle_<i>/pose` (i = 0..`num_dynamic_obstacles`-1, default 6) | `tf2_msgs/TFMessage` | Per-obstacle `PosePublisher` plugin, bridged per-model |
| `/model/dynamic_obstacle_<i>/cmd_vel` | `geometry_msgs/Twist` | `simulation_manager`'s own per-obstacle wander control loop |

### C. Pipeline contract topics (`interfaces` message types, stable across Phase 1/2)

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/perception/detections` | `interfaces/DetectedObjectArray` | `perception_node` | `tracking_node` |
| `/tracking/tracks` | `interfaces/TrackedObjectArray` | `tracking_node` | `motion_prediction`, `visualization_node` (planned) |
| `/prediction/trajectories` | `interfaces/PredictedTrajectoryArray` | `prediction_node` | `risk_node`, `planner_node`, `visualization_node` (planned) |
| `/risk/obstacle_risks` | `interfaces/ObstacleRiskArray` | `risk_node` | `planner_node`, `visualization_node` (planned) |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | `planner_node` | `controller_node` |
| *(proposed, additive, not an `interfaces` message)* `/planner/global_path` | `nav_msgs/Path` | `planner_node` | `visualization_node` (planned) |

Goal-sending reuses `nav2_msgs/action/NavigateToPose` directly, not a custom
action — `planner_node` hosts this action server itself (`navigate_to_pose`),
consistent with Module 7's approved "no real Nav2 bringup" design decision
(§9c, §16). `planner_node` also subscribes to `/prediction/trajectories`
directly (not just `/risk/obstacle_risks`) for obstacle geometry — see §9c's
"clean division of labour" decision.

### D. Placeholder topics (wired, not yet functionally used)

`camera_node` subscribes to `/pi_camera/image_raw` and `lidar_node` subscribes to
`/scan`, but both only log a rate heartbeat in Phase 1 — neither publishes
`DetectedObjectArray`, and `perception_node` does not consume their output.

### E. `/cmd_vel_nav` → `/cmd_vel` / `/cmd_vel_gate` — the actuation bridge

`/cmd_vel_nav` is `planner_node`'s fixed output contract in every phase (§9c), per §11's
"fixed pipeline-contract topic names are module-level constants" rule — not a
Phase-2-only topic. `controller_node` (`robot_controller`, Module 8) is its only
subscriber, and is the one node in the entire pipeline allowed to know which
environment it's running in (§9d, §16): it relays every `/cmd_vel_nav` message onto a
deployment-selected `output_topic` parameter, `/cmd_vel` by default (Phase 1 — Gazebo's
raw diff-drive input, table A) or `/cmd_vel_gate` for Phase 2 (the real BeetleBot's
existing arbitration node, not built by this project). No other node ever needs to
know which one is active.

## 6. Message flow

```
Gazebo ground truth (/model/*/pose[_static], tf2_msgs/TFMessage)
        │  filtered by entity-name prefix (static_obstacle_* / dynamic_obstacle_*)
        ▼
perception_node  @ 10 Hz, confidence pinned to 1.0 (ground truth, no detector uncertainty)
        │
        ▼
/perception/detections  (interfaces/DetectedObjectArray)
        │
        ▼
tracking_node  — synchronous per incoming DetectedObjectArray:
    1. associate   greedy nearest-neighbor, Euclidean gate (default 1.0 m)
    2. predict     every existing track coasted forward by dt (Kalman, constant-velocity)
    3. update      matched tracks corrected with their detection's position
    4. lifecycle   STATUS_TENTATIVE → STATUS_CONFIRMED (3 consecutive hits)
                   STATUS_CONFIRMED/OCCLUDED → STATUS_OCCLUDED (1 consecutive miss)
                   → STATUS_LOST (5 consecutive misses), published once, then dropped
        │
        ▼
/tracking/tracks  (interfaces/TrackedObjectArray: persistent track_id, position,
                   velocity, 6×6 covariance, status, age)
        │
        ▼
prediction_node  — synchronous per incoming TrackedObjectArray:
    1. filter    keep only STATUS_CONFIRMED/STATUS_OCCLUDED tracks (TENTATIVE is
                 unreliable, LOST is about to be dropped)
    2. predict   constant-velocity forward projection, horizon_sec/step_sec
                 points per track (default 3.0s / 0.1s = 30 points), position
                 covariance grown via a locally-reimplemented F/Q propagation
                 (see §9a — not imported from cognitive_tracking)
        │
        ▼
/prediction/trajectories  (interfaces/PredictedTrajectoryArray: per-track
                           model_name="constant_velocity", TrajectoryPoint[]
                           with stamp/position/velocity/growing 3x3 covariance)
        │
        ▼
risk_node  — synchronous per incoming PredictedTrajectoryArray, once /wheel/odom
             has been received at least once:
    1. cache      /wheel/odom position/velocity, twist rotated body→world frame
    2. project    robot's own constant-velocity forecast, sampled at each
                   incoming TrajectoryPoint's own timestamp offset (Phase-1
                   stand-in for a real planned path — dynamic_planner does not
                   exist yet)
    3. score       per track: time_to_collision (first offset within
                   collision_radius_m, else -1.0), path_intersection_prob
                   (closed-form Gaussian falloff on closest-approach distance
                   vs. obstacle covariance + robot_position_std_m), relative_speed
                   (closing speed along the line of sight), distance_to_robot,
                   risk_score (weighted linear combination), threat_level
        │
        ▼
/risk/obstacle_risks  (interfaces/ObstacleRiskArray: per-track risk_score,
                       time_to_collision, path_intersection_prob,
                       relative_speed, distance_to_robot, threat_level)
        │
        ▼
planner_node  — self-hosted nav2_msgs/action/NavigateToPose action server (no
              real Nav2 bringup); timer-driven control loop at control_rate_hz
              once a goal is active:
    1. join       cached PredictedTrajectoryArray (geometry) with cached
                   ObstacleRiskArray (explainable priority), by track_id
    2. gate        emergency stop if any THREAT_CRITICAL track's TTC is within
                   emergency_stop_ttc_sec, overriding candidate scoring
    3. plan        deterministic weighted-scoring local planner
                   (dynamic_planner.local_planner): fixed (v, omega) candidate
                   grid within this cycle's accel-limited dynamic window,
                   each forward-simulated and scored (goal progress, heading,
                   obstacle clearance) if admissible; least-bad max-clearance
                   fallback if boxed in
        │
        ▼
/cmd_vel_nav  (geometry_msgs/Twist, fixed pipeline-contract topic, §5E)
        │
        ▼
controller_node  — the one node allowed to be environment-aware (§9d):
    1. relay      every /cmd_vel_nav message republished immediately onto
                   output_topic ('/cmd_vel' sim default, '/cmd_vel_gate' for
                   Phase 2), push-driven, no transformation
    2. watchdog    independent timer (watchdog_check_rate_hz); publishes a
                   zero Twist if no /cmd_vel_nav has arrived within
                   cmd_timeout_sec -- a last-line-of-defense stop for
                   abnormal upstream silence, distinct from planner_node's
                   own zero-Twist on normal goal completion/cancellation
        │
        ▼
/cmd_vel (Phase 1) or /cmd_vel_gate (Phase 2)  (geometry_msgs/Twist)

(planned) visualization_node subscribes to TrackedObjectArray,
          PredictedTrajectoryArray, and ObstacleRiskArray independently, for
          RViz display — it is not in the control path.
```

## 7. Interfaces overview

`interfaces` (ament_cmake, `rosidl_generate_interfaces`) is the single fixed,
versioned data contract at every pipeline-stage boundary. It is the entire
mechanism behind the contract-first decoupling thesis (§1): every pipeline-stage
package's `package.xml` lists `interfaces` as its only project-internal
dependency, and none lists another stage package. `simulation` is never a build
dependency of any downstream package — it is treated purely as a runtime topic
provider, the same "depend on data shape only, never on another stage's
implementation or build artifacts" principle applied to the sim/hardware
boundary as well as to the pipeline-stage boundaries.

| Message | Published by | Consumed by |
|---|---|---|
| `DetectedObjectArray` | `perception_node` | `tracking_node` |
| `TrackedObjectArray` | `tracking_node` | `motion_prediction`, `visualization_node` |
| `PredictedTrajectoryArray` | `prediction_node` (`motion_prediction`) | `risk_assessment`, `visualization_node` |
| `ObstacleRiskArray` | `risk_assessment` | `dynamic_planner`, `visualization_node` |

Key enums:

| Message | Enum | Values |
|---|---|---|
| `DetectedObject` | `CLASS_*` | `CLASS_UNKNOWN=0`, `CLASS_PEDESTRIAN=1`, `CLASS_DYNAMIC_OBSTACLE=2`, `CLASS_STATIC_OBSTACLE=3` |
| `TrackedObject` | `STATUS_*` | `STATUS_TENTATIVE=0`, `STATUS_CONFIRMED=1`, `STATUS_OCCLUDED=2`, `STATUS_LOST=3` |
| `ObstacleRisk` | `THREAT_*` | `THREAT_LOW=0`, `THREAT_MEDIUM=1`, `THREAT_HIGH=2`, `THREAT_CRITICAL=3` |

Design notes (see `src/interfaces/README.md` for full detail):

- Composed from standard types (`geometry_msgs/Point`, `geometry_msgs/Vector3`,
  `std_msgs/Header`) wherever possible, so RViz/`tf2`/stock ROS2 tooling work on
  these messages without adapters.
- `PredictedTrajectory.model_name` identifies which predictor produced a
  forecast, purely for logging/debugging — the planner must never branch on it.
- `ObstacleRisk` exposes its inputs (TTC, path-intersection probability,
  relative speed, distance) alongside `risk_score`, so risk decisions are
  explainable, not a black box.
- Covariances are flattened row-major arrays, matching `nav_msgs/Odometry` /
  `geometry_msgs/PoseWithCovariance` convention.

## 8. Module 3 implementation details (`cognitive_perception`)

Turns Gazebo ground-truth entity poses into `interfaces/DetectedObjectArray`,
the contract Module 4 consumes.

**Nodes:**

| Node | Phase 1 role | Phase 2 role |
|---|---|---|
| `perception_node` | Reads Gazebo ground truth, publishes `DetectedObjectArray` on `/perception/detections` | Same output contract, fed by `camera_node`/`lidar_node` instead of ground truth |
| `camera_node` | Placeholder: subscribes to `/pi_camera/image_raw`, logs a frame-rate heartbeat only | Runs a real image-based detector (e.g. YOLO) |
| `lidar_node` | Placeholder: subscribes to `/scan`, logs a scan-rate heartbeat only | Runs real LiDAR clustering |

**`perception_node`'s staged design** — five single-purpose methods so a
Phase-2 backend swap only ever touches the input stage:

| Stage | Method | Job |
|---|---|---|
| Input | `_pose_callback` | Cache the latest ground-truth transform per entity |
| Filter | `classify` / `_filter_known_entities` | Decide which cached entities are real obstacles, by name prefix |
| Convert | `_to_detected_object` | One filtered pose → one `DetectedObject` |
| Assemble | `_build_detection_array` | Pure message-building, no I/O — directly unit-testable |
| Output | `_publish_detections` | The only method that touches the publisher |

**Key implementation facts:**

- Subscribes to one topic per obstacle
  (`/model/static_obstacle_<i>/pose_static`, `/model/dynamic_obstacle_<i>/pose`)
  rather than one shared ground-truth topic, because this build's Gazebo
  `PosePublisher` plugin ignores a custom `<topic>` override and always
  publishes per-model; `num_static_obstacles`/`num_dynamic_obstacles` are
  hand-duplicated from `simulation/config/simulation_params.yaml` (no build
  dependency exists to derive them).
- Deliberately does **not** use `/world/<world_name>/pose/info`:
  `gz-sim`'s `SceneBroadcaster` never sets the per-pose header data
  (`frame_id`/`child_frame_id`) the `ros_gz_bridge` `Pose→TransformStamped`
  conversion needs, so `child_frame_id` would come through empty and nothing
  would be classifiable by name.
- Publishing is decoupled from the obstacle pose message rate: detections are
  built and published on a fixed `publish_rate_hz` timer (default 10 Hz) from
  whatever is currently cached, giving `cognitive_tracking` a predictable
  cadence independent of Gazebo's own update rate.
- Bounding-box size is approximated from config parameters
  (`static_obstacle_size`, `dynamic_obstacle_diameter`/`_height`), not measured,
  since the ground-truth pose topics report position only.
- `confidence` is hardcoded to `1.0` for this node's entire Phase-1 lifetime —
  ground truth has no detector uncertainty to report. The field is still
  populated (not left at zero) so nothing downstream ever special-cases
  "ground truth mode."
- `detection_frame_id` (default `"world"`) is applied unconditionally to the
  outgoing header, independent of the incoming `TransformStamped.header`, since
  the incoming frame name is Gazebo's SDF world name
  (`"multi_obstacle_arena"`), not the pipeline's expected frame.
- No build dependency on `simulation` — treats it as a pure runtime topic
  source (§7).

**Testing:** `test_perception_node.py` (6 tests) exercises the pose
cache/classification/message-building methods directly; `test_sensor_placeholders.py`
(2 tests) exercises `camera_node`/`lidar_node`'s heartbeat counters. Neither
touches Gazebo. Manual verification via `ros2 topic hz`/`echo` against a live
Gazebo instance is documented separately in `src/cognitive_perception/README.md`.

## 9. Module 4 implementation details (`cognitive_tracking`)

Turns `interfaces/DetectedObjectArray` into `interfaces/TrackedObjectArray` —
persistent, ID-stable, Kalman-smoothed tracks — the contract Module 5 and
`visualization_node` consume.

**Node:** `tracking_node`, subscribing to `/perception/detections` and
publishing `TrackedObjectArray` on `/tracking/tracks` **synchronously**, once
per incoming `DetectedObjectArray` (no independent prediction timer in Phase 1).

**Staged design** — mirrors `cognitive_perception`'s convention:

| Stage | Method | Job |
|---|---|---|
| Input | `_detections_callback` | Entry point; computes `dt` and drives every stage below in order |
| Association | `_associate` | Greedy nearest-neighbor over a Euclidean cost matrix, matched against each track's **pre-predict** position |
| Kalman Predict | `_predict` | Coasts every existing track forward by `dt`, matched or not |
| Kalman Update | `_update` | Corrects only matched tracks with their associated detection's position |
| Lifecycle | `_manage_lifecycle` | Drives each `Track`'s `STATUS_*` state machine; spawns new tentative tracks for unmatched detections |
| Assemble | `_build_tracked_array` | Pure message-building, no I/O |
| Output | `_publish` | The only method that touches the publisher |

**Supporting modules** (each free of ROS/rclpy imports, independently
unit-testable):

- `kalman_filter.py` — `KalmanFilter6D`: constant-velocity model over state
  `[x, y, z, vx, vy, vz]`, full 6×6 covariance maintained throughout, `predict(dt)`
  and `update(measurement)` kept as separate methods (not one combined step).
- `association.py` — `build_cost_matrix()` (Euclidean distances,
  `(num_tracks, num_detections)` shape) and `greedy_nearest_neighbor()`
  (global closest-pair-first matching, not row-by-row).
- `track.py` — `Track` dataclass owning `register_hit()`/`register_miss()`,
  the lifecycle state machine.

**Lifecycle thresholds** (all ROS parameters, `config/tracking_params.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `confirm_after_hits` | 3 | Consecutive hits before `TENTATIVE` → `CONFIRMED` |
| `occluded_after_misses` | 1 | Consecutive misses before `CONFIRMED`/`OCCLUDED` → `OCCLUDED` |
| `lost_after_misses` | 5 | Consecutive misses before → `STATUS_LOST` (published once, then dropped) |

**Other parameters:** `gating_distance_m` (1.0), `process_noise_std` (0.5),
`measurement_noise_std` (0.1), `initial_velocity_variance` (10.0), `min_dt_sec`
(0.001, guards the predict step against a zero/negative `dt` from
duplicate/out-of-order timestamps).

**Key implementation facts:**

- Association runs against each track's *last known* position, not a
  predicted-forward position — pipeline order is Association → Predict →
  Update, not the more common Predict → Association → Update. A deliberate
  Phase-1 simplification traded for a simpler, independently-testable
  association stage; negligible at `perception_node`'s 10 Hz rate.
- Greedy nearest-neighbor, not the Hungarian algorithm — `association.py`'s
  functions take a `(cost_matrix, gating_threshold) →
  (matches, unmatched_tracks, unmatched_detections)` shape so an
  optimal-assignment replacement later is a function swap, not a
  `tracking_node` rewrite.
- Gating is flat Euclidean distance, not Mahalanobis — `KalmanFilter6D` still
  maintains the full covariance regardless, so Mahalanobis gating is a pure
  `_associate`-level change later, not a data-model change.
- `class_id`/`size` are overwritten from the newest matched detection every
  cycle — no voting or confidence fusion in Phase 1.
- `STATUS_LOST` tracks are included in the array once (matching
  `TrackedObject.msg`'s own "about to be dropped by the tracker" comment),
  then removed from the internal track list only *after* publishing.
- NumPy (`python3-numpy`) is this workspace's first non-ROS scientific
  dependency, confined to `kalman_filter.py`/`association.py`.

**Testing:** 27 pytest tests across four files
(`test_kalman_filter.py`, `test_association.py`, `test_track.py`,
`test_tracking_node.py`), all built on fabricated inputs — no Gazebo, no live
ROS graph. Additionally verified live during implementation: `tracking_node`
run standalone and fed real `DetectedObjectArray` messages over
`ros2 topic pub`/`ros2 topic echo`, confirming correct behavior end-to-end
through the actual ROS graph, not just direct method calls.

## 9a. Module 5 implementation details (`motion_prediction`)

Turns `interfaces/TrackedObjectArray` into `interfaces/PredictedTrajectoryArray` —
a constant-velocity forecast trajectory per stable tracked object — the contract
Module 6 (`risk_assessment`) and `visualization_node` consume.

**Node:** `prediction_node`, subscribing to `/tracking/tracks` and publishing
`PredictedTrajectoryArray` on `/prediction/trajectories` **synchronously**, once
per incoming `TrackedObjectArray` (no independent timer, mirroring `tracking_node`).

**Staged design** — mirrors `cognitive_perception`/`cognitive_tracking`'s convention:

| Stage | Method | Job |
|---|---|---|
| Input | `_tracks_callback` | Entry point; drives every stage below in order |
| Filter | `_select_predictable_tracks` | Keeps only `STATUS_CONFIRMED`/`STATUS_OCCLUDED` tracks |
| Predict | `_predict_trajectory` | Per-track constant-velocity forward projection (`motion_prediction.trajectory_predictor`) |
| Assemble | `_build_trajectory_array` | Pure message-building, no I/O |
| Output | `_publish` | The only method that touches the publisher |

**Supporting module** (free of ROS/rclpy imports, independently unit-testable):

- `trajectory_predictor.py` — `predict_trajectory()`: repeatedly applies a
  constant-velocity state-transition (`F`) and process-noise (`Q`) matrix,
  identical in form to `cognitive_tracking.kalman_filter.KalmanFilter6D.predict()`
  but **reimplemented locally**, `horizon_sec / step_sec` times, starting from a
  `TrackedObject`'s published position/velocity/6×6 covariance. Returns one
  `(offset_sec, position, velocity, position_covariance)` tuple per step; the
  3×3 position-covariance block is what fills `TrajectoryPoint.covariance`.

**Parameters** (all ROS parameters, `config/prediction_params.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `horizon_sec` | 3.0 | How far into the future to forecast each track (seconds) |
| `step_sec` | 0.1 | Spacing between forecast points (seconds) — 30 points/track at the defaults |
| `process_noise_std` | 0.5 | Process-noise std for this package's own covariance propagation — independently declared, not shared with `cognitive_tracking`'s parameter of the same name |

**Key implementation facts:**

- Only `STATUS_CONFIRMED` and `STATUS_OCCLUDED` tracks are forecast.
  `STATUS_TENTATIVE` tracks have too little history to trust, and
  `STATUS_LOST` tracks are already about to be dropped by `tracking_node` —
  forecasting either would be forecasting noise. `STATUS_OCCLUDED` tracks are
  included deliberately: their coasted Kalman estimate is still worth
  projecting forward while the tracker waits to reconfirm or drop them.
- The constant-velocity forecast math is **deliberately duplicated**, not
  imported, from `cognitive_tracking.kalman_filter.KalmanFilter6D.predict()` —
  same `F`/`Q` construction, independent implementation — so
  `motion_prediction` has no build/runtime dependency on `cognitive_tracking`
  (§7). A future change to one filter's noise model does not silently change
  the other's forecast.
- `process_noise_std` is declared independently in
  `motion_prediction/config/prediction_params.yaml`, not derived from or
  synced with `cognitive_tracking/config/tracking_params.yaml`'s parameter of
  the same name — consistent with this workspace's existing
  deliberate-duplication convention (§13's `num_static_obstacles` example).
- `model_name` is hardcoded to `"constant_velocity"` on every published
  `PredictedTrajectory`, purely for logging/debugging per
  `interfaces/msg/PredictedTrajectory.msg`'s own comment — `risk_assessment`
  must never branch on it.
- Publishing is synchronous with track arrival, not its own timer — same
  reasoning as `tracking_node` (Module 4 already publishes at a fixed cadence,
  so a second independent timer would add complexity without benefit in
  Phase 1).
- No build dependency on `cognitive_tracking` — treats `/tracking/tracks` as a
  pure runtime topic source, consistent with §7's interfaces-only dependency
  rule.

**Testing:** 15 pytest tests across two files (`test_trajectory_predictor.py`,
6 tests; `test_prediction_node.py`, 9 tests), all built on fabricated inputs —
no Gazebo, no live ROS graph, no `tracking_node`. Additionally verified live
during implementation: `prediction_node` run standalone and fed a fabricated
`TrackedObjectArray` over `ros2 topic pub`/`ros2 topic echo`, confirming
correct `model_name`, point count, and growing covariance end-to-end through
the actual ROS graph.

## 9b. Module 6 implementation details (`risk_assessment`)

Turns `interfaces/PredictedTrajectoryArray` (Module 5) and the robot's own `/wheel/odom`
into `interfaces/ObstacleRiskArray` — an explainable, per-obstacle collision-risk score —
the contract `dynamic_planner` (Module 7) and `visualization_node` consume.

**Node:** `risk_node`, subscribing to `/prediction/trajectories` and `/wheel/odom`,
publishing `ObstacleRiskArray` on `/risk/obstacle_risks` **synchronously**, once per
incoming `PredictedTrajectoryArray` (no independent timer, mirroring `tracking_node`/
`prediction_node`). No risk is published until at least one `/wheel/odom` message has
been cached.

**Staged design** — mirrors `cognitive_perception`/`cognitive_tracking`/`motion_prediction`'s
convention:

| Stage | Method | Job |
|---|---|---|
| Input (cache) | `_odom_callback` | Caches the robot's current position/velocity, twist rotated from body frame into world frame. The only method touching the odometry subscriber. |
| Input (drive) | `_trajectories_callback` | Entry point; drives every stage below in order. The only method touching the trajectories subscriber. |
| Compute | `_compute_risk` | Per-track risk scoring (`risk_assessment.risk_model`) |
| Assemble | `_build_risk_array` | Pure message-building, no I/O |
| Output | `_publish` | The only method that touches the publisher |

**Supporting module** (free of ROS/rclpy imports, independently unit-testable):

- `risk_model.py` — `assess_obstacle_risk()`: combines one obstacle's trajectory (as
  `(offset_sec, position, velocity, position_covariance)` tuples — the same per-step shape
  `trajectory_predictor.predict_trajectory()` returns) with the robot's own
  constant-velocity-projected position at each of those same offsets into
  `time_to_collision`, `path_intersection_prob`, `relative_speed`, `distance_to_robot`,
  `risk_score`, and `threat_level`.

**Parameters** (all ROS parameters, `config/risk_params.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `robot_radius_m` | 0.2 | Summed into `collision_radius_m` for `time_to_collision` |
| `obstacle_radius_m` | 0.4 | Fixed generic obstacle radius — see limitations below |
| `robot_position_std_m` | 0.1 | Stand-in for real robot localization uncertainty |
| `weight_ttc` / `weight_prob` / `weight_speed` / `weight_distance` | 0.35 / 0.35 / 0.15 / 0.15 | `risk_score` component weights |
| `max_relative_speed_mps` | 3.0 | Closing speed at/above which the speed component saturates |
| `max_distance_m` | 5.0 | Distance at/beyond which the distance component is zero |
| `threat_medium_min` / `threat_high_min` / `threat_critical_min` | 0.25 / 0.5 / 0.75 | `risk_score` cut points for `threat_level` |

**Key implementation facts:**

- The robot's own future path is a locally-projected constant-velocity forecast from
  cached `/wheel/odom`, not a real planned path — `dynamic_planner` (Module 7) does not
  exist yet. This is a documented Phase-1 stand-in, approved as a design decision before
  implementation, replaced by Nav2's real plan in Module 7 without `risk_node`'s published
  contract changing.
- Every trajectory sample's time offset is read directly from `TrajectoryPoint.stamp`
  (`point.stamp - header.stamp`), not from a separately configured `horizon_sec`/`step_sec`
  — `risk_assessment` never needs to know `motion_prediction`'s sampling parameters to stay
  correct.
- Obstacle radius (`obstacle_radius_m`) is a single fixed configured constant, the same for
  every track, because `PredictedTrajectory` carries no size/class_id field. Subscribing to
  `/tracking/tracks` for real per-track size was considered and rejected, to keep
  `risk_node`'s input exactly as documented in §5C (`/prediction/trajectories` only).
- `path_intersection_prob` is a closed-form Gaussian falloff
  (`exp(-0.5 * (min_distance / combined_std)^2)`) on the closest-approach distance, combining
  the obstacle's own propagated position covariance with `robot_position_std_m`. Deterministic,
  no RNG/Monte Carlo, no new dependency (no SciPy `erf` needed, consistent with §17's
  "defer SciPy" stance).
- `risk_score` is a weighted **linear** combination of four normalized components (TTC,
  path-intersection probability, closing speed, distance) — deliberately inspectable rather
  than a black-box function, per §10's "clean architecture over sophistication."
- `distance_to_robot` and `relative_speed` are computed against a trajectory's *first*
  point (one sample interval into the future), not a true `t=0` sample — `PredictedTrajectory`
  has no `t=0` point. A documented Phase-1 proxy, negligible at `motion_prediction`'s default
  0.1s step.
- `/wheel/odom`'s `twist` is rotated from the body frame into the world frame using the
  odometry message's own orientation quaternion (`rotate_vector_by_quaternion`), since
  `nav_msgs/Odometry`'s twist convention is body-frame, not the frame its `pose` is in.
- No build dependency on `motion_prediction` or `cognitive_tracking` — treats
  `/prediction/trajectories`/`/wheel/odom` as pure runtime topic sources, consistent with
  §7's interfaces-only dependency rule.

**Testing:** 25 pytest tests across two files (`test_risk_model.py`, 17 tests;
`test_risk_node.py`, 8 tests), all built on fabricated inputs — no Gazebo, no live ROS
graph, no `motion_prediction`. Additionally verified live during implementation: `risk_node`
run standalone and fed a fabricated `Odometry` then `PredictedTrajectoryArray` over
`ros2 topic pub`/`ros2 topic echo`, confirming correct `distance_to_robot`, `relative_speed`,
`time_to_collision`, `path_intersection_prob`, `risk_score`, and `threat_level` for a direct
collision-course obstacle end-to-end through the actual ROS graph.

## 9c. Module 7 implementation details (`dynamic_planner`)

Hosts a `nav2_msgs/action/NavigateToPose` action server and turns `interfaces/ObstacleRiskArray`,
`interfaces/PredictedTrajectoryArray`, and the robot's own `/wheel/odom` into velocity
commands on `/cmd_vel_nav` — the fixed contract `robot_controller` (Module 8) consumes.

**Node:** `planner_node`. Subscribes to `/risk/obstacle_risks`, `/prediction/trajectories`,
`/wheel/odom` (all cache-only). Hosts the `navigate_to_pose` action server. Runs a
**timer-driven control loop** (`control_rate_hz`, default 10 Hz) that publishes `/cmd_vel_nav`
once per cycle whenever a goal is active — the first deliberate deviation in this workspace
from every prior node's synchronous-per-message convention (§16), because a velocity-command
publisher cannot go silent between upstream messages the way a pass-through stage can.
Publishes `nav_msgs/Path` on `/planner/global_path` once per accepted goal, for
explainability/RViz only (not part of the control path).

**Staged design** — mirrors every prior module's convention, extended for the action-server +
timer shape:

| Stage | Method | Job |
|---|---|---|
| Input (cache) | `_odom_callback` | Robot position/yaw/body-frame velocity |
| Input (cache) | `_risks_callback` | Latest `ObstacleRiskArray`, by `track_id` |
| Input (cache) | `_trajectories_callback` | Latest `PredictedTrajectoryArray`, by `track_id` |
| Input (goal) | `_goal_callback`/`_cancel_callback`/`_execute_callback` | Standard `NavigateToPose` action-server handlers |
| Control (drive) | `_control_loop` | Timer callback; entry point for every stage below, once per cycle while a goal is active |
| Join | `_join_obstacles` | Combines cached trajectories (geometry) with cached risks (priority), by `track_id` |
| Plan | `_compute_command` | `dynamic_planner.local_planner.select_command()` |
| Assemble | `_build_cmd_vel`/`_build_global_path` | Pure message-building, no I/O |
| Output | `_publish_cmd`/`_publish_global_path` | The only methods that touch their respective publishers |

**Supporting modules** (each free of ROS/rclpy imports, independently unit-testable):

- `local_planner.py` — `select_command()`: a small, fixed, deterministically-ordered grid of
  `(v, omega)` candidates within the current cycle's accel-limited dynamic window, each
  forward-simulated with simple unicycle kinematics (`simulate_unicycle`) over
  `local_horizon_sec`. Candidates clear of every obstacle by more than `collision_radius_m`
  across the whole horizon are "admissible" and scored (`score_candidate`) by a weighted
  linear combination of goal progress, heading alignment, and clearance; the max-clearance
  candidate is returned instead (flagged `admissible=False`) if none are admissible (boxed in).
  No random sampling anywhere — the same inputs always produce the same command.
- `global_path.py` — `generate_straight_line_path()`: evenly-spaced waypoints from the robot's
  current position to the goal, for `/planner/global_path` only. `local_planner.py` scores
  directly against the goal position, not path waypoints — a straight line's heading is
  identical everywhere along it, so an explicit lookahead mechanism would add no behavior.

**Parameters** (all ROS parameters, `config/planner_params.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `control_rate_hz` | 10.0 | Fixed-rate control loop frequency |
| `max_linear_speed_mps` / `max_angular_speed_radps` | 0.5 / 1.5 | Kinematic limits |
| `max_linear_accel_mps2` / `max_angular_accel_radps2` | 0.5 / 2.0 | Define each cycle's dynamic window |
| `num_linear_samples` / `num_angular_samples` | 5 / 7 | Candidate `(v, omega)` grid density |
| `local_horizon_sec` / `local_step_sec` | 1.5 / 0.1 | Per-candidate forward-simulation horizon/spacing |
| `robot_radius_m` / `obstacle_radius_m` | 0.2 / 0.4 | Summed into `collision_radius_m`; independently declared from `risk_assessment`'s parameters of the same name |
| `safety_margin_m` | 1.0 | Distance beyond `collision_radius_m` at which the clearance component saturates |
| `weight_progress` / `weight_heading` / `weight_clearance` | 0.5 / 0.2 / 0.3 | `local_planner` score component weights |
| `goal_tolerance_m` | 0.15 | Distance to goal at/below which `NavigateToPose` succeeds |
| `emergency_stop_ttc_sec` | 0.5 | `THREAT_CRITICAL` + TTC at/below this forces an immediate stop |
| `waypoint_spacing_m` | 0.5 | Spacing for the visualization-only global path |

**Key implementation facts:**

- **No real Nav2 bringup.** `planner_node` hosts the `NavigateToPose` action server itself —
  no `bt_navigator`, no lifecycle-managed costmap/planner/controller servers, no C++
  `nav2_costmap_2d` plugin. Approved design decision: a real Nav2 costmap layer is a C++
  pluginlib plugin, which would have been this workspace's first C++/`ament_cmake`
  node-bearing package (every other node-bearing package is `ament_python`, §2/§11) purely to
  satisfy a Phase-1 reference implementation. `/cmd_vel_nav` and the `NavigateToPose` action
  are the fixed published interfaces a real Nav2 stack can replace this package's internals
  behind later, without either changing.
- **Deterministic weighted-scoring, not MPPI.** §15's original wording sketched an "MPPI
  controller" before any planning module existed. MPPI is inherently stochastic
  (randomly-sampled control rollouts), in direct tension with "deterministic behavior" being
  an explicit Module 7 priority — approved decision to use the deterministic scorer described
  above instead, matching `risk_assessment.risk_model`'s own explainable-linear-combination
  philosophy.
- **Obstacle geometry from `/prediction/trajectories`, risk from `/risk/obstacle_risks` — a
  deliberate division of labour, not redundancy.** `ObstacleRisk.msg` carries no position field
  (its own header comment: "not a raw sensor measurement"), so `local_planner.py` cannot do
  per-candidate geometric clearance checking from it alone. `planner_node._join_obstacles`
  combines both by `track_id`; `local_planner.py` itself only ever sees geometry
  (`ObstacleView`) — `threat_level`/`time_to_collision` are used exclusively as a coarser
  **emergency-stop safety gate** (`_emergency_stop_triggered`), never folded into per-candidate
  scoring. Approved decision, precedented by `visualization_node`'s own documented independent
  multi-topic subscription pattern (§6/§15) — no `interfaces` message was changed.
- **Straight-line global path, no map/SLAM.** Phase 1 has no map or localization stack
  anywhere in this pipeline, only odometry — a real Nav2-style global planner isn't
  meaningfully available. `global_path.py` is a documented Phase-1 stand-in, same spirit as
  `risk_node`'s odometry self-projection (§9b).
- **`MultiThreadedExecutor`, not `rclpy.spin()`.** A direct consequence of the timer-driven
  control loop: the action server's long-running `_execute_callback` (blocks in a poll loop
  until the goal succeeds/cancels) and the fixed-rate `_control_loop` timer must run
  concurrently, which a single-threaded executor cannot do without deadlocking. `main()` uses
  `MultiThreadedExecutor` with the action server and the timer/subscriptions on separate
  `MutuallyExclusiveCallbackGroup`s — the first deviation in this workspace from §11's
  `rclpy.init → rclpy.spin → destroy_node/shutdown` pattern; see the amended note in §11.
- **One active goal at a time.** `_goal_callback` rejects a new goal while one is already
  executing rather than preempting it — the simplest correct Phase-1 behavior; a real Nav2
  stack's preemption semantics can replace this later.
- **`nav2_msgs` is this workspace's first non-`ament_python`-authored ROS message dependency
  from outside this project** (previously only stock `geometry_msgs`/`nav_msgs`/`std_msgs`
  etc. were used, all standard ROS2 core packages). It was not installed in the development
  environment and required a manual `sudo apt install ros-jazzy-nav2-msgs` — see §17.
- **No build dependency on `risk_assessment` or `motion_prediction`.** Treats
  `/risk/obstacle_risks`/`/prediction/trajectories`/`/wheel/odom` as pure runtime topic
  sources, consistent with §7's interfaces-only dependency rule.

**Testing:** 36 pytest tests across three files (`test_global_path.py`, 5 tests;
`test_local_planner.py`, 18 tests; `test_planner_node.py`, 13 tests), all built on fabricated
inputs — no Gazebo, no live ROS graph, no `risk_assessment`/`motion_prediction`. `ros-jazzy-nav2-msgs`
was installed after implementation (§17); `colcon build`/`colcon test` then ran clean workspace-wide
(117 tests, 0 new failures). Additionally verified live during implementation, across three
scenarios — `planner_node` run standalone, fed a fabricated `Odometry` over `ros2 topic pub` and a
goal via `ros2 action send_goal`:

1. **Far, obstacle-free goal** — `/planner/global_path` published exactly once with the expected
   start/end waypoints; `/cmd_vel_nav` published the expected dynamic-window-limited forward
   command (`linear.x=0.05`, matching hand-derived expectations for a from-rest first cycle) at
   `control_rate_hz`.
2. **Emergency-stop gate** — a `THREAT_CRITICAL` risk with `time_to_collision=0.1` on
   `/risk/obstacle_risks` forced `/cmd_vel_nav` to all-zero for the entire observation window,
   despite a far, otherwise-clear goal and no obstacle geometry on `/prediction/trajectories` at
   all — confirms the gate acts independently of `local_planner.py`'s candidate scoring.
3. **Goal already within `goal_tolerance_m`** — the action reported `SUCCEEDED` immediately, with
   `error_code: 0`.

## 9d. Module 8 implementation details (`robot_controller`)

Relays `geometry_msgs/Twist` commands from `/cmd_vel_nav` (`dynamic_planner`'s fixed output,
every phase) onto a deployment-selected output topic — Gazebo's raw `/cmd_vel` in Phase 1, the
real BeetleBot's existing `/cmd_vel_gate` arbitration node in Phase 2. The final Phase-1
module; closes the loop from planning into actual actuation.

**Node:** `controller_node`, subscribing to `/cmd_vel_nav` and republishing immediately
(push-driven, no message transformation) onto `output_topic`. Runs an independent
watchdog timer (`watchdog_check_rate_hz`) that publishes a zero `Twist` if no
`/cmd_vel_nav` message has arrived within `cmd_timeout_sec`. Plain `rclpy.spin()` — no
blocking callback exists here, so unlike `planner_node` (§9c) a `MultiThreadedExecutor`
is not needed.

**Staged design** — thinner than every prior module, since a `Twist`-to-`Twist` relay has no
transformation and therefore no "assemble" stage:

| Stage | Method | Job |
|---|---|---|
| Input | `_cmd_vel_nav_callback` | Relays immediately; records arrival time for the watchdog |
| Safety | `_watchdog_check` | Timer callback; publishes a zero `Twist` if `/cmd_vel_nav` has gone stale |
| Output | `_publish_command` | The only method that touches the output publisher |

**Parameters** (all ROS parameters, `config/controller_params.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `output_topic` | `/cmd_vel` | Deployment-selected relay destination — override to `/cmd_vel_gate` for Phase 2 |
| `cmd_timeout_sec` | 0.5 | Watchdog: publish a zero `Twist` if `/cmd_vel_nav` has been silent this long |
| `watchdog_check_rate_hz` | 10.0 | How often the watchdog checks for staleness |

**Key implementation facts:**

- **`output_topic` is a parameter, the one deliberate exception to §11's "topic names are
  never ROS parameters" rule.** Every other topic name is a module-level constant because it's
  a pipeline-stage-boundary contract that a deployment must never be able to silently break;
  `output_topic` is a different kind of thing entirely — the sim/hardware bridge selection
  point (§5A/§5E), meant to be reconfigured per deployment. Approved design decision.
- **Command-staleness watchdog, not a pure relay.** `planner_node` already publishes an
  explicit zero `Twist` on normal `NavigateToPose` success/cancellation (§9c), but that only
  covers the well-behaved case — if `/cmd_vel_nav` goes silent abnormally (a `planner_node`
  crash, a network partition), most Gazebo diff-drive plugins (and plausibly a real motor
  controller) keep applying the last received `Twist` indefinitely, not just once. The
  watchdog is a distinctly-placed last-line-of-defense safety measure appropriate for the
  actuation bridge specifically, firing only once per stale period (reset by the next real
  command) to avoid redundant re-stops. Approved design decision, directly serving "safety"
  as an explicit priority for this final module.
- **No independent velocity clamping.** `dynamic_planner` already enforces its own kinematic
  limits (§9c); duplicating that check here was considered and rejected as redundant scope
  creep for a module whose job is bridging, not planning. Approved design decision.
- **No dependency on `interfaces` at all** — the first pipeline-stage package with no use for
  any custom message type, since relaying `geometry_msgs/Twist` requires no transformation.
  No build dependency on `dynamic_planner` either — `/cmd_vel_nav` is consumed purely as a
  runtime topic, consistent with §7's interfaces-only dependency rule.
- **Does not implement `/cmd_vel_gate`'s arbitration logic itself.** `robot_controller` only
  publishes *into* `/cmd_vel_gate` on real hardware; the arbitration node listening there
  (e.g. reconciling autonomous commands against a manual teleop/E-stop override) is existing
  BeetleBot infrastructure this project does not build or modify.

**Testing:** 8 pytest tests in `test_controller_node.py`, all built on fabricated `Twist`
messages — no Gazebo, no live ROS graph, no `dynamic_planner`. Watchdog staleness is
simulated by rewinding `node._last_received_time` with `rclpy.duration.Duration` rather than
sleeping in real time, keeping the suite instant and deterministic. Additionally verified live
during implementation, across three scenarios — `controller_node` run standalone, fed
fabricated `Twist` messages over `ros2 topic pub`:

1. **Relay correctness** — each `/cmd_vel_nav` message reappeared immediately on `/cmd_vel`.
2. **Watchdog** — real silence exceeding `cmd_timeout_sec` (including the natural startup
   latency between separate `ros2 topic pub` invocations, which alone exceeded the configured
   timeout) reliably produced a zero `Twist` on `/cmd_vel`, with relay resuming correctly on
   the next fresh command.
3. **`output_topic:=/cmd_vel_gate` override** — confirmed via `ros2 node info /controller_node`
   to redirect the publisher, proving the Phase-2 deployment path without any code change.

## 10. Design philosophy

- **Contract-first decoupling.** The `interfaces` package is the only shared
  dependency between pipeline stages. This is what makes "swap the tracker
  backend without touching the predictor" an enforceable property, not just an
  aspiration.
- **Digital-twin parity.** `simulation` exposes the real robot's documented
  topic names/types, so no node needs a sim/hardware branch.
- **Staged, single-purpose node methods.** Every node in this workspace
  factors its work into a repeating shape: an *input* stage, one or more
  *processing* stages (filter/associate/convert/predict/update/classify), an
  *assemble* stage that is pure and I/O-free, and an *output* stage that is the
  only place a publisher is touched. This shape recurs deliberately across
  `perception_node` and `tracking_node` so that (a) each stage is independently
  unit-testable, and (b) a future backend swap has an obvious, narrow surface
  to change.
- **Clean architecture over algorithmic sophistication, in Phase 1.**
  `cognitive_tracking`'s explicit goal is to be the reference implementation a
  future DeepSORT/ByteTrack backend replaces — not to be the most accurate
  tracker possible today. Greedy association and Euclidean gating are
  deliberate, documented, revisitable simplifications, not oversights.
- **Config over code for anything that might plausibly change.** Frame IDs,
  gating thresholds, lifecycle counts, obstacle sizes — all parameters, not
  literals, so a tuning change is a one-line YAML edit.
- **Explicit, documented simplifications, never silent ones.** Every
  simplification in this codebase has an accompanying comment or README note
  explaining why it exists and what would need to change to remove it (see
  §14, §16, §17).

## 11. Coding conventions

- Python 3.12 / `rclpy`, `ament_python` for every node-bearing package
  (`interfaces` is the sole `ament_cmake` exception, for message generation).
- One node class per file, named `<Thing>Node`, at
  `<package_name>/<package_name>/<node_name>.py`, with a plain `main(args=None)`
  entry point (`rclpy.init` → `rclpy.spin` → `destroy_node`/`shutdown` in a
  `try`/`finally`). **Exception:** `dynamic_planner`'s `planner_node` uses
  `rclpy.executors.MultiThreadedExecutor` instead of plain `rclpy.spin()`,
  because its action server's long-running execute callback and its
  fixed-rate control-loop timer must run concurrently (§9c, §16) — the only
  node in this workspace with that requirement so far.
- Node logic is factored into staged, private (`_leading_underscore`) methods
  with one clear entry point — usually a subscription callback or a timer —
  that orchestrates them in a fixed order (§10). Pure "assemble" methods that
  build a message are kept separate from "output" methods that touch a
  publisher, specifically so message-building is unit-testable without a live
  publisher.
- Type hints are used throughout, including PEP 585 generics
  (`list[str]`, `dict[str, Transform]`, `tuple[float, float, float]`) — no
  `typing.List`/`typing.Tuple` needed on this Python version.
- Fixed pipeline-contract topic names are module-level constants
  (e.g. `DETECTIONS_TOPIC`, `TRACKS_TOPIC`), never bare string literals and
  never ROS parameters — a stage-boundary topic name is part of the
  `interfaces` contract, not something a deployment should reconfigure.
  **Exception:** `robot_controller`'s `controller_node` declares its output
  destination (`output_topic`) as a ROS parameter, not a constant — it is
  the sim/hardware bridge selection point (§5E), not a pipeline-stage
  boundary, and is deliberately meant to be reconfigured per deployment
  (§9d, §16).
- Plain state-holding types use `@dataclass` (e.g. `Track`), not hand-rolled
  `__init__`/`__eq__`.
- No premature abstraction: there is no base-class/plugin interface for
  "detectors" or "trackers" ahead of a second implementation existing. A
  backend swap is expected to rewrite a module's internals directly, not to
  implement an interface that has only ever had one implementer.
- Comments explain **why**, never restate **what** — a workaround, a
  non-obvious constraint, or a fact confirmed against a real install (e.g. "a
  real Harmonic install"), not a restatement of the following line of code.

## 12. Testing conventions

- `pytest`, run via `colcon test` / `colcon test-result --verbose`.
- `rclpy.init()`/`rclpy.shutdown()` are called manually inside each test
  function's `try`/`finally` — there is no shared fixture yet.
- Tests call a node's staged/private methods **directly** (e.g.
  `node._detections_callback(msg)`) rather than exercising the live ROS graph
  — no rosbag, no real topics, no Gazebo required for any automated test in
  this workspace.
- Pure-logic modules (`kalman_filter.py`, `association.py`, `track.py`) are
  tested with plain data (numpy arrays, dataclass instances) and have zero
  ROS/rclpy dependency in their own test files.
- Node-level tests fabricate the relevant message type directly (helper
  functions like `_detection(...)`, `_detections_array(...)`) and either
  inspect the node's internal state afterward or monkey-patch the publisher
  (`node._pub.publish = list.append`) to capture what would have been sent.
- `ament_lint_auto`/`ament_lint_common` are **not** wired into any
  `ament_python` package's `package.xml` in this workspace — only `interfaces`
  (`ament_cmake`) uses them. This is a deliberate, consistent choice, not an
  oversight per package.
- Each package README documents a **Manual** verification step (`ros2 topic
  hz`/`echo`, `ros2 node info`) separately from the **Automated** `colcon test`
  step, since automated tests never touch Gazebo or a live ROS graph.
- Before a module is considered complete, it should also be smoke-tested
  through the actual ROS graph at least once (node run standalone, fed via
  `ros2 topic pub`, observed via `ros2 topic echo`) — the unit-test suite
  alone does not prove the node's `rclpy` wiring (QoS, topic names, message
  types) is correct end-to-end. See §14 for the current gap: no *multi-node*
  integration test exists yet.

## 13. Configuration conventions

- Every tunable value is a declared ROS parameter
  (`self.declare_parameter(...)` in `__init__`), read once via a local
  `gp = self.get_parameter` alias.
- One `config/<package>_params.yaml` per package, one top-level key per node
  name, using the standard `ros__parameters` YAML convention, loaded by that
  package's launch file — never hardcoded in the node.
- Fixed pipeline-contract topic names are **not** parameters (§11) — changing
  one would break a neighboring package's expectations, which is not something
  a deployment-time reconfiguration should be able to do silently.
- Some parameters are **deliberately duplicated** across a package boundary
  rather than derived or imported — e.g. `num_static_obstacles`/
  `num_dynamic_obstacles` are declared independently in both
  `simulation/config/simulation_params.yaml` and
  `cognitive_perception/config/perception_params.yaml` — specifically to avoid
  creating a build dependency between packages that must stay decoupled (§7).
- `use_sim_time` is passed as a launch argument (default `true`) to every
  node via the standard ROS2 mechanism; no node explicitly
  `declare_parameter`s it.
- YAML comments explain the **rationale** for a default value (e.g. "midpoint
  of static_obstacle_min_size/max_size"), not just the key name.

## 14. Known limitations

- Association (`cognitive_tracking`) gates on each track's pre-predict
  position, not its predicted-forward position — acceptable at
  `perception_node`'s 10 Hz rate; would need revisiting for much lower
  detection rates or much faster obstacles (§9, §17).
- Greedy nearest-neighbor association is not globally optimal — dense or
  crossing-track scenarios can produce a suboptimal match compared to the
  Hungarian algorithm. Deliberately deferred, not an oversight (§10, §16).
- No Mahalanobis gating — the gate is a flat Euclidean distance even though
  `KalmanFilter6D` already maintains the full covariance needed for it.
- No class voting/confidence fusion in `tracking_node` — `class_id` is
  overwritten by the newest matched detection every cycle. Only safe because
  Phase 1's ground-truth detector never misclassifies; a real Phase-2 detector
  backend would need this addressed.
- Bounding-box size in `DetectedObject`/`TrackedObject` is approximated from
  config-provided spawn ranges, not measured — the ground-truth pose topics
  report position only.
- `camera_node`/`lidar_node` are placeholders: wired to real sensor topics but
  perform no detection; `perception_node` does not consume their output in
  Phase 1.
- `tracking_node` publishes synchronously with detection arrival and has no
  predict-only coasting timer — if `perception_node` stops publishing (sensor
  dropout), tracks freeze rather than being predicted forward and marked
  occluded, until a new detection cycle resumes. Deferred by design decision
  (§16), acceptable because Module 3 currently publishes reliably at a fixed
  10 Hz.
- The `gz-sim-velocity-control-system` plugin filename used for dynamic
  obstacles is confirmed on the current development install but not
  guaranteed across every Gazebo Harmonic point release — see
  `src/simulation/README.md`'s "If obstacles don't move" section.
- No automated **multi-node** integration test exists (e.g. `perception_node`
  and `tracking_node` running together under `colcon test`). The two have
  only been verified together manually (see §9's "Testing" note and §12).
- `risk_node`'s obstacle radius (`obstacle_radius_m`) is a single fixed config
  value, not each obstacle's real size — `PredictedTrajectory` carries no
  size/class_id field to derive it from (§9b). `time_to_collision` is
  therefore only as accurate as that generic default.
- `risk_node` projects the robot's own future path via constant-velocity
  extrapolation from `/wheel/odom`, not a real planned/global path — there is
  no such topic until `dynamic_planner` (Module 7) exists. A robot that turns
  sharply within the prediction horizon will have this self-projection
  diverge from its actual path, the same limitation §17 already documents for
  `motion_prediction`'s obstacle forecasts.
- `risk_node`'s `path_intersection_prob` combines obstacle covariance with a
  fixed `robot_position_std_m` config value, not a real robot localization
  covariance — Phase 1 has no localization uncertainty estimate to read yet.
- `risk_node` assumes `/wheel/odom`'s pose is expressed in the same frame as
  `/prediction/trajectories` (`"world"`, per `perception_node`'s
  `detection_frame_id`) — no TF lookup or transform is performed between them.
  If Phase 2's real odometry publishes in a different frame, this would need
  revisiting.
- `planner_node`'s `_join_obstacles` silently skips any track present in
  `/prediction/trajectories` but not yet in the cached `/risk/obstacle_risks`
  (or vice versa) that cycle, rather than planning around it blind — a
  documented Phase-1 simplification, negligible given both topics are
  published from the same upstream cadence (§9c).
- `planner_node`'s obstacle-forecast time offsets are computed against each
  received `PredictedTrajectoryArray`'s own `header.stamp`, but the control
  loop runs on an independent timer decoupled from message arrival — unlike
  `risk_node`, which only ever compares trajectories at the moment they
  arrive. This introduces a small, bounded clock skew between "now" in the
  candidate simulation and "now" in the cached obstacle forecast, up to one
  upstream publish period. Not corrected for in Phase 1.
- `dynamic_planner` has no awareness of the arena's outer walls or any other
  untracked static geometry — `perception_node` only classifies
  `static_obstacle_*`/`dynamic_obstacle_*` entities (§8), never the arena
  boundary itself, and no map/occupancy-grid topic exists anywhere in this
  pipeline. A goal placed such that the straight-line path or a locally
  "clear" candidate would exit the arena is not guarded against. Pre-existing
  pipeline-wide gap, not introduced by Module 7, but only became reachable
  once a planner existed to act on it.
- `planner_node` accepts only one active `NavigateToPose` goal at a time,
  rejecting a new goal rather than preempting the current one — a real Nav2
  stack's preemption semantics are not replicated in Phase 1.
- `controller_node`'s watchdog only detects *silence* on `/cmd_vel_nav`, not
  malformed or out-of-range content in a message that does arrive — it
  performs no independent velocity clamping (§9d, an approved decision, not
  an oversight), so a future planner backend that forgets its own kinematic
  limits would not be caught at this layer.
- `robot_controller` does not implement `/cmd_vel_gate`'s arbitration logic
  — it only publishes into that topic on real hardware. The real BeetleBot's
  arbitration node (reconciling autonomous commands against, e.g., a manual
  teleop/E-stop override) is existing infrastructure this project assumes
  exists but does not build, test, or verify (§9d, §17).
- With Module 8 complete, the pipeline's `/cmd_vel_nav` → `/cmd_vel`
  bootstrapping gap noted for Modules 6/7 (their outputs having no live
  subscriber yet) is now closed — see §5E.
- This document (`PROJECT_CONTEXT.md`) did not exist prior to Module 4's
  completion, despite being referenced by relative path in
  `cognitive_perception/cognitive_perception/perception_node.py`'s comments.
  This is the first canonical version; any prior informal notes it may have
  superseded are not preserved here.

## 15. Planned integration work

All eight numbered modules (`cognitive_perception` through `robot_controller`) are
complete — see §8, §9, §9a, §9b, §9c, §9d for implementation details. The full Phase-1
pipeline runs end-to-end, perception through actuation.

**Unnumbered integration package — `cognitive_bringup`.** Top-level launch
files, RViz configs, world files, and `visualization_node` (subscribes to
`TrackedObjectArray`, `PredictedTrajectoryArray`, and `ObstacleRiskArray`
independently, for display — not part of the control path).

## 16. Important architectural decisions made during implementation

- `interfaces` was created first, with zero project-specific dependencies, so
  every other package can depend on it without any risk of a dependency
  cycle.
- `nav2_msgs/action/NavigateToPose` is reused for goal-sending rather than a
  custom action — no reason to reinvent what Nav2 already provides.
- `simulation` exposes the real robot's documented topic names/types (digital
  twin) so no downstream node needs a sim/hardware branch.
- Per-obstacle `PosePublisher` plugin + per-model topic
  (`/model/<name>/pose[_static]`) was chosen over the world's shared
  `/world/<world>/pose/info`, because `SceneBroadcaster`'s `Pose_V` never
  populates the header data the `tf2_msgs` bridge conversion needs — confirmed
  against a real Harmonic install, not assumed from documentation.
- `perception_node` reads ground truth directly in Phase 1 rather than
  routing through `camera_node`/`lidar_node`; those nodes exist purely so
  their topic wiring is already correct ahead of Phase 2.
- `perception_node` has no build dependency on `simulation` — treated purely
  as a runtime topic provider, consistent with the interfaces-only dependency
  rule (§7).
- `perception_node`'s `detection_frame_id` is hardcoded to `"world"` and
  applied unconditionally, rather than copied from the incoming transform's
  frame, because the incoming frame name is Gazebo's SDF world name, not the
  pipeline's expected frame.
- `cognitive_tracking` introduces NumPy as this workspace's first non-ROS
  scientific dependency, confined to `kalman_filter.py`/`association.py`,
  chosen over introducing SciPy at the same time.
- Greedy nearest-neighbor was chosen over the Hungarian algorithm for Module
  4's association stage, with the cost-matrix/match-function boundary
  designed so an optimal-assignment replacement later is a drop-in function
  swap, not a `tracking_node` rewrite.
- Association is performed against each track's pre-predict position, not its
  predicted-forward position — trading a small amount of accuracy at high
  frame rates for a simpler, independently-testable association stage.
- Track lifecycle thresholds (`confirm_after_hits=3`,
  `occluded_after_misses=1`, `lost_after_misses=5`) are consecutive-count
  based and reset on any opposite outcome, not cumulative counters.
- `STATUS_LOST` tracks are published exactly once before being pruned, so no
  consumer ever observes a track disappear from `/tracking/tracks` with no
  terminal status reported.
- `tracking_node` publishes synchronously per incoming `DetectedObjectArray`
  rather than on its own timer, deferring a predict-only coasting timer until
  it is actually needed (Module 3 already publishes reliably at 10 Hz).
- `class_id`/`size` on a `TrackedObject` are overwritten by the newest matched
  detection with no voting, deferring classification fusion until a real
  (fallible) detector backend exists to make voting meaningful.
- `motion_prediction` forecasts only `STATUS_CONFIRMED`/`STATUS_OCCLUDED`
  tracks, never `STATUS_TENTATIVE` (too little history) or `STATUS_LOST`
  (already about to be dropped by `tracking_node`).
- `motion_prediction`'s constant-velocity covariance propagation
  (`trajectory_predictor.py`) reimplements the same `F`/`Q` construction as
  `cognitive_tracking.kalman_filter.KalmanFilter6D.predict()` rather than
  importing it, so the two packages stay architecturally decoupled (§7) even
  though the underlying math is deliberately equivalent.
- `motion_prediction` declares its own `process_noise_std` parameter
  independently of `cognitive_tracking`'s parameter of the same name, rather
  than sharing or deriving it — consistent with this workspace's existing
  deliberate-duplication convention (§13).
- `prediction_node` publishes synchronously per incoming `TrackedObjectArray`,
  with no independent forecasting timer, mirroring `tracking_node`'s
  synchronous-publish decision (Module 4 already publishes at a fixed
  cadence, so a second timer here would add complexity without benefit in
  Phase 1).
- `risk_node` projects the robot's own future path via local constant-velocity
  extrapolation from `/wheel/odom`, rather than consuming a real planned-path
  topic, because `dynamic_planner` (Module 7) does not exist yet. Approved as
  a Phase-1 design decision before implementation (see §9b); replaced by
  Nav2's real plan in Module 7 without `risk_node`'s published contract
  changing.
- `risk_node`'s obstacle radius is a single fixed configured constant
  (`obstacle_radius_m`), the same for every track, rather than either adding
  a size field to `PredictedTrajectory` or subscribing to `/tracking/tracks`
  directly for real per-track size — both alternatives were considered and
  rejected to keep `risk_node`'s only meaningful input exactly as documented
  in §5C (`/prediction/trajectories` only), with no interface change and no
  dependency on a non-adjacent stage's topic.
- `risk_node`'s `path_intersection_prob` uses a closed-form Gaussian falloff
  on covariance (`exp(-0.5 * (min_distance / combined_std)^2)`) rather than a
  simple geometric distance threshold, specifically to make use of the
  per-point position covariance `PredictedTrajectory` already exposes and
  that §17 says `risk_assessment` should treat as the mechanism for
  interpreting forecast uncertainty. Deterministic and closed-form, not
  sampled, to avoid introducing RNG/Monte Carlo or a new dependency (SciPy).
- `risk_node`'s `risk_score` is a weighted **linear** combination of four
  normalized components (TTC, path-intersection probability, closing speed,
  distance), not a nonlinear/black-box function, consistent with §10's
  "clean architecture over algorithmic sophistication" and
  `interfaces/README.md`'s explainability design note.
- Every trajectory sample's time offset in `risk_node` is derived directly
  from each `TrajectoryPoint.stamp` rather than a separately configured
  `horizon_sec`/`step_sec` parameter — avoids `risk_assessment` needing to
  know or duplicate `motion_prediction`'s sampling parameters to stay
  correct, an even looser coupling than the deliberate-duplication
  convention used elsewhere (§13).
- `risk_node` publishes synchronously per incoming `PredictedTrajectoryArray`,
  with no independent timer, mirroring `tracking_node`/`prediction_node`'s
  synchronous-publish decision, and additionally withholds publishing
  entirely until at least one `/wheel/odom` message has been received.
- `dynamic_planner`'s `planner_node` hosts the `NavigateToPose` action server
  itself rather than relying on a real Nav2 bringup (`bt_navigator`,
  lifecycle-managed servers, a C++ `nav2_costmap_2d` plugin) — approved
  Phase-1 design decision, avoiding this workspace's first C++/`ament_cmake`
  node-bearing package and a heavy bringup dependency (§9c).
- `local_planner.py` uses a deterministic weighted-scoring candidate scorer
  (fixed grid, no random sampling) rather than a true stochastic MPPI
  controller, even though §15 originally sketched "an MPPI controller" before
  any planning module existed — approved because MPPI's randomness is in
  direct tension with "deterministic behavior" being an explicit Module 7
  priority (§9c).
- `planner_node` reads obstacle geometry from `/prediction/trajectories`
  directly, joined by `track_id` with `/risk/obstacle_risks`, rather than
  adding a position field to `ObstacleRisk.msg` — keeps the frozen
  `interfaces` contract unchanged (no message has been modified since Module
  1) and keeps `ObstacleRisk.msg` true to its own header comment ("not a raw
  sensor measurement"). Precedented by `visualization_node`'s already-documented
  independent multi-topic subscription pattern (§6/§15/§9c).
- `ObstacleRiskArray`'s `threat_level`/`time_to_collision` are used by
  `planner_node` only as a coarse emergency-stop safety gate, never folded
  into `local_planner.py`'s per-candidate scoring — a deliberate division of
  labour between the risk-assessment stage (explainable priority) and the
  planning stage (geometric clearance), approved alongside the previous
  decision (§9c).
- `dynamic_planner`'s global path (`global_path.py`) is a straight line from
  the robot's current position to the goal, not a real Nav2-style
  map-searching global planner — there is no map/SLAM/localization stack
  anywhere in this pipeline in Phase 1, only odometry, so a real global
  planner isn't meaningfully available yet (§9c).
- `planner_node`'s control loop runs on its own fixed-rate timer
  (`control_rate_hz`) rather than publishing synchronously per incoming
  message, breaking the synchronous-publish convention every prior node
  (Modules 4-6) deliberately followed — approved because a velocity-command
  publisher cannot go silent between upstream messages the way a
  pass-through stage safely can (§9c).
- `planner_node`'s `main()` uses `rclpy.executors.MultiThreadedExecutor`
  instead of the `rclpy.spin()` pattern every other node in this workspace
  uses — a direct, necessary consequence of the previous decision: the
  action server's execute callback and the control-loop timer must run
  concurrently. Amended into §11's coding conventions as the first
  documented exception to that pattern.
- `planner_node` accepts only one active `NavigateToPose` goal at a time
  (rejects a new goal rather than preempting), the simplest correct Phase-1
  behavior, deferring real preemption semantics to a future Nav2 stack.
- `robot_controller`'s `output_topic` is a declared ROS parameter rather than
  a fixed module-level constant like every other topic in this workspace —
  approved as the one deliberate exception to §11's "topic names are never
  parameters" rule, because it is the sim/hardware bridge selection point,
  not a pipeline-stage-boundary contract (§9d).
- `controller_node` includes an independent command-staleness watchdog
  (publishes a zero `Twist` if `/cmd_vel_nav` goes silent past
  `cmd_timeout_sec`) rather than being a pure relay — approved because
  `planner_node`'s own zero-Twist only covers normal goal
  completion/cancellation, leaving abnormal upstream silence (a crash, a
  network partition) unhandled otherwise (§9d).
- `robot_controller` performs no independent velocity clamping, even though
  it is the final actuation gate — considered and rejected as redundant
  scope creep given `dynamic_planner` already enforces its own kinematic
  limits; a bridging module duplicating a planning module's safety checks
  was judged to add a second place those limits must be kept in sync, not
  genuine defense-in-depth (§9d).
- `controller_node` uses plain `rclpy.spin()`, not the `MultiThreadedExecutor`
  `planner_node` needs — its subscription callback and watchdog timer are
  both non-blocking, so §11's executor exception does not apply here.

## 17. Assumptions and rationale behind each design choice

- **Assumes `perception_node`'s publish rate stays fixed and reasonably high
  (~10 Hz).** If it drops much lower or becomes bursty, both Module 4's
  synchronous-publish-no-timer design and its pre-predict association
  ordering become less accurate and should be revisited (§14).
- **Assumes ground-truth detection confidence is always 1.0 and `class_id` is
  always correct in Phase 1.** Both `tracking_node`'s no-voting class handling
  and `perception_node`'s hardcoded confidence rely on this. A real Phase-2
  detector backend breaks both assumptions and must update the one
  corresponding line in each module (documented at that exact line).
- **Assumes obstacles rarely cluster within the gating distance
  (`gating_distance_m`, default 1.0 m) of one another.** Greedy nearest-
  neighbor's suboptimality versus the Hungarian algorithm only becomes visible
  when two tracks' gates overlap around the same detection; `simulation`'s own
  `min_entity_separation: 1.0` parameter is set with this in mind.
- **Assumes the real BeetleBot's sensor topics
  (`/scan`, `/imu/data`, `/pi_camera/image_raw`, `/wheel/odom`) match the
  simulated bridge's names and types exactly.** This is the entire basis for
  the "runs unmodified on hardware" claim in the top-level README; any
  real-robot topic rename breaks that claim for every downstream package
  simultaneously.
- **Assumes no pipeline-stage package will ever add another pipeline-stage
  package as a project-internal dependency.** This is what makes "swap
  `tracking_node`'s backend without touching `motion_prediction`" actually
  true. A stage that starts importing symbols from a neighboring stage's
  package (rather than only subscribing to its topic) would silently
  invalidate the entire justification for `interfaces` existing as a separate
  package (§7).
- **Assumes Python 3.12 / `rclpy` on ROS2 Jazzy**, which is what makes the
  PEP 585 generic type hints (`list[str]`, `tuple[float, float, float]`) used
  throughout valid without importing from `typing`. An earlier Python version
  would need `typing.List`/`typing.Tuple` instead.
- **Assumes NumPy is available and acceptable on every target platform**,
  including the Raspberry Pi 5 BeetleBot, not just the Gazebo development
  machine. Confirmed available in the current development environment
  (NumPy 1.26.4); Phase 2 deployment should confirm it is present on the Pi's
  image before relying on `cognitive_tracking` there.
- **Assumes no other package will need a heavier optimization dependency
  (e.g. SciPy) before Module 4's association algorithm is next revisited.** If
  a future module needs SciPy anyway, the "defer the Hungarian algorithm"
  decision in Module 4 should be reconsidered rather than treated as fixed
  policy — the reason to defer it was avoiding a new dependency, not that
  greedy matching is believed to be sufficient forever.
- **Assumes obstacle motion is well-approximated by a constant-velocity model
  over `motion_prediction`'s 3-second default horizon.** A real obstacle that
  turns, accelerates, or decelerates within that window will have its forecast
  trajectory diverge from its actual path; `risk_assessment` (Module 6) treats
  `PredictedTrajectory`'s per-point covariance (which grows with horizon) as
  the mechanism for expressing this growing uncertainty via
  `path_intersection_prob`, not the forecast position as exact. Revisit if
  Module 6 needs tighter forecasts for fast-turning obstacles — the
  `model_name` field exists precisely so a learned model can replace this
  assumption later without an interface change.
- **Assumes the robot's own motion is also well-approximated by a
  constant-velocity model over the same horizon**, since `risk_node` has no
  real planned path to consume yet (`dynamic_planner`/Module 7 does not
  exist). A robot executing a sharp turn mid-horizon will have `risk_node`'s
  self-projection diverge from its actual path, understating or overstating
  risk for that window. Revisit once Module 7 publishes a real plan
  `risk_node` can consume instead (§9b, §16).
- **Assumes every obstacle can be approximated by the same fixed
  `obstacle_radius_m`, regardless of its real size or class.**
  `PredictedTrajectory` carries no size/class_id field to do better with in
  Phase 1 (§9b). A real Phase-2 detector/tracker backend that publishes
  meaningfully different obstacle sizes should prompt revisiting whether
  `interfaces/msg/PredictedTrajectory.msg` needs a size field, rather than
  `risk_assessment` continuing to guess with one constant.
- **Assumes `/wheel/odom`'s pose is published in the same frame as
  `/prediction/trajectories`** (`"world"`) **and that `robot_position_std_m`
  is an adequate stand-in for real localization uncertainty.** Phase 1 has no
  localization stack publishing a real pose covariance; Phase 2 deployment
  onto the physical BeetleBot should confirm both the frame assumption and
  replace the fixed `robot_position_std_m` with a real covariance once one
  exists.
- **Assumes `ros-jazzy-nav2-msgs` (and, in Phase 2, the rest of the Nav2
  message/action packages a real Nav2 stack would need) is installed on every
  target platform**, including the Raspberry Pi 5 BeetleBot, not just the
  Gazebo development machine. It was not present in this workspace's
  development environment when `dynamic_planner` was first implemented, and
  had to be added manually (`sudo apt install ros-jazzy-nav2-msgs`) — the
  first ROS message package this workspace depends on that isn't a standard
  core package already present in a base ROS2 install. Confirmed installed
  and working (§9c's "Testing" note) as of the live verification pass; Phase
  2 deployment should still confirm it is present on the Pi's image before
  relying on `dynamic_planner` there.
- **Assumes the robot's forward/angular speed reported on `/wheel/odom`'s
  `twist.twist.linear.x`/`twist.twist.angular.z` are body-frame values usable
  directly as `local_planner.py`'s unicycle-model `(v, omega)`**, consistent
  with `nav_msgs/Odometry`'s documented convention. If Phase 2's real
  odometry source reports these fields differently, `planner_node._odom_callback`
  is the one place that would need to change (§9c).
- **Assumes a straight line to the goal is an acceptable stand-in for a real
  global plan given this pipeline has no map/SLAM/localization stack.** A
  goal on the far side of a wall or a large obstacle cluster has no route
  around it in Phase 1 — `local_planner.py`'s local deviation can dodge
  individual tracked obstacles but cannot discover a detour around static
  geometry the straight line passes through. Revisit once a real map-aware
  global planner exists (§9c, §14).
- **Assumes obstacle-forecast time offsets and the control loop's own "now"
  stay closely enough synchronized that the bounded clock skew between them
  (§14) doesn't matter at `control_rate_hz`'s default 10 Hz.** Would need
  revisiting if either the control rate drops much lower or upstream
  publishing becomes bursty, the same category of assumption §17 already
  makes about `perception_node`'s publish rate for Module 4.
- **Assumes the real BeetleBot's `/cmd_vel_gate` accepts `geometry_msgs/Twist`**,
  the same type as `/cmd_vel_nav` and `/cmd_vel` — `robot_controller`'s relay
  performs no message-type conversion. If the real arbitration node expects a
  different message type or an additional priority/source field,
  `controller_node._publish_command`'s output construction is the one place
  that would need to change (§9d).
- **Assumes a Gazebo diff-drive plugin (and plausibly a real motor
  controller) holds the last received `Twist` indefinitely rather than
  requiring periodic re-publication to keep moving.** This is the entire
  motivation for `controller_node`'s watchdog (§9d) — if the actual actuation
  layer instead has its own built-in command-timeout/failsafe, the watchdog
  becomes redundant defense-in-depth rather than the only safeguard, which is
  still an acceptable outcome but worth confirming against the real
  BeetleBot's motor controller behavior in Phase 2.
- **Assumes the real BeetleBot's `/cmd_vel_gate` arbitration node already
  exists, is already correct, and needs no changes from this project** —
  `robot_controller` only ever publishes into it, per `PROJECT_CONTEXT.md`'s
  original Module 8 description. If Phase 2 discovers `/cmd_vel_gate` doesn't
  exist yet or behaves differently than documented here, that is
  BeetleBot-platform work outside this pipeline's packages, not a
  `robot_controller` change.
