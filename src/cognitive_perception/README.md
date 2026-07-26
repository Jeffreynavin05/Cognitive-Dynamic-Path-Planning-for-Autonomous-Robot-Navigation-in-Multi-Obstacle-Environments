# cognitive_perception

Module 3 of the cognitive navigation pipeline: turns raw perception input into
`interfaces/DetectedObjectArray`, the fixed contract `cognitive_tracking` (Module 4)
consumes. Phase 1 detects from Gazebo ground truth; `camera_node`/`lidar_node` are
Phase-2 placeholders, wired to their real sensor topics but not yet doing detection.

## Why this package exists

Per the top-level README's pipeline (`camera_node`/`lidar_node` → `perception_node` →
`tracking_node` → ...), this package owns everything upstream of tracking. Keeping
the detector *implementation* separate from the `interfaces/DetectedObjectArray`
*contract* is what lets Phase 1's ground-truth detector be swapped for a real
YOLO/LiDAR-clustering pipeline in Phase 2 without `cognitive_tracking` (or anything
downstream) changing at all.

## Nodes

| Node | Phase 1 role | Phase 2 role |
|---|---|---|
| `perception_node` | Reads Gazebo ground-truth entity poses, publishes `DetectedObjectArray` on `/perception/detections` | Same output contract, but fed by `camera_node`/`lidar_node` instead of ground truth |
| `camera_node` | Placeholder: subscribes to `/pi_camera/image_raw`, logs a frame-rate heartbeat only | Runs the real image-based detector (e.g. YOLO) |
| `lidar_node` | Placeholder: subscribes to `/scan`, logs a scan-rate heartbeat only | Runs real LiDAR clustering |

`perception_node` does **not** depend on `camera_node`/`lidar_node` in Phase 1 --
ground truth is read directly, matching the "ground-truth detection → YOLO" swap
described in the top-level README and the explicit hand-off in `simulation/README.md`
("[filtering ground truth by name prefix] is Module 3's job, not this package's").

## Design notes

- **`perception_node` is staged into five single-purpose methods, not one callback
  that does everything**, so a Phase-2 swap only ever touches one stage:

  | Stage | Method | Job |
  |---|---|---|
  | Input | `_pose_callback` | Cache the latest ground-truth transform per entity. Phase 2 replaces *only* this stage with whatever `camera_node`/`lidar_node` feed in. |
  | Filter | `classify` / `_filter_known_entities` | Decide which cached entities are real obstacles. |
  | Convert | `_to_detected_object` | Turn one filtered pose into one `DetectedObject`. |
  | Assemble | `_build_detection_array` | Build the full `DetectedObjectArray`. Pure message-building, no I/O -- directly unit-testable without a publisher. |
  | Output | `_publish_detections` | The timer callback; the only method that touches the publisher. |

  Everything from *filter* onward is detection-source-agnostic: it only ever reads
  `self._latest_poses`, never the obstacle pose topic directly, so replacing the input
  stage with a real camera/lidar pipeline in Phase 2 doesn't require touching filter,
  convert, assemble, or output at all.
- **`perception_node` subscribes to one topic per obstacle**: `/model/static_obstacle_<i>/pose_static`
  for each of `num_static_obstacles` and `/model/dynamic_obstacle_<i>/pose` for each of
  `num_dynamic_obstacles` (`tf2_msgs/msg/TFMessage`, bridged by
  `simulation/launch/world.launch.py`, one bridge line per topic) -- see
  `_obstacle_pose_topics()`. It keeps the latest transform per entity name in memory,
  filtering to only `static_obstacle_*` → `CLASS_STATIC_OBSTACLE` and
  `dynamic_obstacle_*` → `CLASS_DYNAMIC_OBSTACLE` (see `simulation/README.md`'s naming
  convention). This is deliberately **not** `/world/<world_name>/pose/info`: that topic
  comes from `gz-sim`'s `SceneBroadcaster`, whose `Pose_V` sets each entity's `name` but
  never the per-pose header data (`frame_id`/`child_frame_id`) that the `ros_gz_bridge`
  `Pose→TransformStamped` conversion reads -- so every `child_frame_id` bridged from
  `pose/info` comes through as an empty string and nothing is classifiable. Each
  obstacle instead carries its own `gz-sim-pose-publisher-system` plugin
  (`simulation_manager.OBSTACLE_POSE_PUBLISHER_PLUGIN`), which *does* set that header
  data. One topic per obstacle rather than one shared topic because this build's
  `PosePublisher` was confirmed, on a real Harmonic install, to ignore a custom
  `<topic>` override and always publish on its per-model default
  (`/model/<name>/pose`, or `/model/<name>/pose_static` for a `static_publisher`
  entity) -- hence `num_static_obstacles`/`num_dynamic_obstacles` are duplicated into
  `perception_params.yaml` just to know how many per-model topics to subscribe to.
  Everything else that could theoretically appear in a pose stream -- the robot
  itself, arena walls, the ground plane -- was never bridged onto these topics in the
  first place, rather than being silently skipped downstream.
- **Publishing is decoupled from the obstacle pose message rate.** Detections are built
  and published on a fixed `publish_rate_hz` timer (default 10 Hz) from whatever is
  currently cached, rather than republishing on every incoming pose message -- this
  gives `cognitive_tracking` a predictable cadence independent of Gazebo's own update
  rate.
- **Bounding-box size is approximated, not measured.** The obstacle pose topics only
  report entity *position*, not extent, so `DetectedObject.size` is filled from
  config parameters that mirror `simulation`'s spawn ranges
  (`static_obstacle_min/max_size`, `dynamic_obstacle_radius`) rather than each
  entity's actual randomized per-run size. Acceptable for ground-truth Phase 1, where
  `confidence` is already fixed at `1.0`; a real detector would measure this properly.
- **`confidence` is hardcoded to `1.0` in `_to_detected_object`, deliberately, for
  this node's entire Phase-1 lifetime.** Ground truth is read straight out of
  Gazebo's own simulation state, not inferred by a model, so there's no detector
  uncertainty to report -- `1.0` isn't a stand-in default, it's the objectively
  correct value while the "detector" is the simulator itself. The field is still
  populated (not left at zero) precisely so nothing downstream ever has to
  special-case "ground truth mode": `interfaces/DetectedObject.confidence` exists as
  a variable 0.0-1.0 field because Phase 2 replaces that one line with the real
  detector backend's per-object score (e.g. a YOLO class-confidence output or a
  LiDAR cluster's fit quality), and `cognitive_tracking` already expects to consume
  a value that varies.
- **`detection_frame_id` is set directly on the outgoing `DetectedObjectArray.header`,
  independent of whatever is on the incoming `TransformStamped.header`.** Confirmed on
  a real Harmonic install: the per-model pose topics' `header.frame_id` now comes
  through as `"multi_obstacle_arena"` (the world name, via `PosePublisher`'s
  `frame_id` header-data entry -- unlike `/world/.../pose/info`, which left it empty
  before this fix), not `"world"`. `detection_frame_id` still defaults to `"world"`
  and is applied unconditionally rather than copied from the input, since the position
  *values* are in world coordinates either way and `"world"` is `cognitive_tracking`'s
  expected frame name, not the world's SDF name. Still a config parameter so it's a
  one-line fix in `config/perception_params.yaml` if that convention ever changes, not
  a code change.
- **No build dependency on `simulation`.** Per `PROJECT_CONTEXT.md` §7, this package
  only depends on `interfaces` for message types and treats `simulation` purely as a
  runtime topic provider -- consistent with the "digital twin" decoupling principle.
- **`camera_node`/`lidar_node` are intentionally thin.** They exist now so the node
  names, executable entries, and topic wiring already match the top-level README's
  pipeline diagram before Phase 2 needs real detection logic here; they do not publish
  `DetectedObjectArray` or feed `perception_node` in Phase 1.

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces simulation cognitive_perception
source install/setup.bash
```

## Run

```bash
# terminal 1
ros2 launch simulation world.launch.py

# terminal 2, once Gazebo and the obstacles have spawned
ros2 launch cognitive_perception perception.launch.py
```

## Test

**Automated (pure logic, no Gazebo needed):**
```bash
colcon test --packages-select cognitive_perception
colcon test-result --verbose
```

**Manual (confirm real detections flow end-to-end):**
```bash
ros2 topic hz /perception/detections
ros2 topic echo /perception/detections --once
```
You should see one `DetectedObject` per spawned static/dynamic obstacle (4 static +
`num_dynamic_obstacles` dynamic, per `simulation/config/simulation_params.yaml`),
each with `confidence: 1.0` and `class_id` `3` (static) or `2` (dynamic).

Confirm the placeholders are alive without doing real detection:
```bash
ros2 topic hz /pi_camera/image_raw /scan   # camera_node/lidar_node log the same rates
ros2 node info /camera_node
ros2 node info /lidar_node                  # no published topics on either, by design
```
