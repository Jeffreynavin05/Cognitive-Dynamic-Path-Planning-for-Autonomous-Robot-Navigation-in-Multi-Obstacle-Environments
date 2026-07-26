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
10. [Design philosophy](#10-design-philosophy)
11. [Coding conventions](#11-coding-conventions)
12. [Testing conventions](#12-testing-conventions)
13. [Configuration conventions](#13-configuration-conventions)
14. [Known limitations](#14-known-limitations)
15. [Planned Modules 6–8](#15-planned-modules-68)
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
              (Module 3, done)   (Module 4, done)  (Module 5, done)  (Module 6, planned) (Module 7, planned) (Module 8, planned)
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
| `risk_assessment` | ament_python (planned) | planned (Module 6) | Per-obstacle collision risk scoring (TTC, path intersection, relative speed, distance) |
| `dynamic_planner` | ament_python (planned) | planned (Module 7) | Nav2 integration, custom risk-aware costmap layer, MPPI controller |
| `robot_controller` | ament_python (planned) | planned (Module 8) | Bridges planner output to `/cmd_vel_nav`, respects the real robot's `/cmd_vel_gate` arbitration |
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
| `risk_assessment` | planned (Module 6) | — | Not started |
| `dynamic_planner` | planned (Module 7) | — | Not started |
| `robot_controller` | planned (Module 8) | — | Not started |
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
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | Sim's raw diff-drive input. **Not** the real robot's Nav2 output topic — see `/cmd_vel_nav` below |

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
| `/prediction/trajectories` | `interfaces/PredictedTrajectoryArray` | `prediction_node` | `risk_assessment` (planned), `visualization_node` (planned) |
| *(planned)* `/risk/obstacle_risks` | `interfaces/ObstacleRiskArray` | `risk_assessment` | `dynamic_planner`, `visualization_node` |

Goal-sending reuses `nav2_msgs/action/NavigateToPose` directly, not a custom
action.

### D. Placeholder topics (wired, not yet functionally used)

`camera_node` subscribes to `/pi_camera/image_raw` and `lidar_node` subscribes to
`/scan`, but both only log a rate heartbeat in Phase 1 — neither publishes
`DetectedObjectArray`, and `perception_node` does not consume their output.

### E. Real-robot-only topics (Phase 2 target, not present in sim)

`/cmd_vel_nav` — the real output topic `dynamic_planner`/Nav2 will publish to
(Module 7/8). On hardware this passes through the BeetleBot's existing
`/cmd_vel_gate` arbitration node; that node does not exist in simulation, where
`/cmd_vel` is used directly instead.

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
(planned) risk_assessment — combines each PredictedTrajectoryArray with the
          robot's own planned/current path to produce an ObstacleRiskArray:
          time_to_collision, path_intersection_prob, relative_speed,
          distance_to_robot, and the resulting risk_score/threat_level
        │
        ▼
(planned) dynamic_planner — ObstacleRiskArray feeds a custom risk-aware costmap
          layer into Nav2 + an MPPI controller; goal-sending via
          nav2_msgs/action/NavigateToPose
        │
        ▼
(planned) robot_controller — relays planner output to /cmd_vel_nav (hardware) /
          /cmd_vel (sim)

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
  `try`/`finally`).
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
- This document (`PROJECT_CONTEXT.md`) did not exist prior to Module 4's
  completion, despite being referenced by relative path in
  `cognitive_perception/cognitive_perception/perception_node.py`'s comments.
  This is the first canonical version; any prior informal notes it may have
  superseded are not preserved here.

## 15. Planned Modules 6–8

Module 5 (`motion_prediction`) is complete — see §9a for its implementation
details.

**Module 6 — `risk_assessment`.** Consumes `PredictedTrajectoryArray` (and,
implicitly, the robot's own planned/current path or pose), publishes
`interfaces/ObstacleRiskArray`. Per `interfaces/README.md`'s explainability
design note, must compute and expose `time_to_collision`,
`path_intersection_prob`, `relative_speed`, and `distance_to_robot` as
first-class fields alongside the final `risk_score`/`threat_level` — not just
the score.

**Module 7 — `dynamic_planner`.** Nav2 integration: a custom risk-aware
costmap layer consuming `ObstacleRiskArray`, plus an MPPI controller.
Goal-sending reuses `nav2_msgs/action/NavigateToPose` directly — no custom
action.

**Module 8 — `robot_controller`.** Bridges the planner's output to
`/cmd_vel_nav` (not `/cmd_vel`), so it passes through the real robot's
existing `/cmd_vel_gate` arbitration node on hardware. In simulation this
arbitration node does not exist, so `/cmd_vel` is used directly there instead
— `robot_controller`'s relay is what reconciles the two without any upstream
node needing to know which environment it's in.

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
  trajectory diverge from its actual path; `risk_assessment` (Module 6) should
  treat `PredictedTrajectory`'s per-point covariance (which grows with
  horizon) as the mechanism for expressing this growing uncertainty, not treat
  the forecast position as exact. Revisit if Module 6 needs tighter forecasts
  for fast-turning obstacles — the `model_name` field exists precisely so a
  learned model can replace this assumption later without an interface change.
