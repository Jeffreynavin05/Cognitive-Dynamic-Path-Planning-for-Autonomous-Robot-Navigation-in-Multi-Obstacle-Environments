# cognitive_tracking

Module 4 of the cognitive navigation pipeline: turns `interfaces/DetectedObjectArray`
(ID-less, per-frame detections from `cognitive_perception`) into
`interfaces/TrackedObjectArray` -- persistent, ID-stable, Kalman-smoothed tracks --
the fixed contract `motion_prediction` (Module 5) and `visualization_node` consume.

## Why this package exists

Per the top-level README's pipeline (`perception_node` -> `tracking_node` ->
`prediction_node` -> ...), this package owns everything between raw detection and
motion forecasting. Keeping the tracker *implementation* separate from the
`interfaces/TrackedObjectArray` *contract* is what lets this project's Kalman
tracker be swapped for DeepSORT/ByteTrack later without `motion_prediction` (or
anything downstream) changing at all -- see `interfaces/msg/TrackedObject.msg`'s
own header comment.

## Node

| Node | Role |
|---|---|
| `tracking_node` | Subscribes to `/perception/detections`, publishes `TrackedObjectArray` on `/tracking/tracks` synchronously, once per incoming `DetectedObjectArray` |

## Design notes

- **`tracking_node` is staged into distinct methods, not one callback that does
  everything**, mirroring `cognitive_perception`'s convention so a future backend
  swap only ever touches one stage:

  | Stage | Method | Job |
  |---|---|---|
  | Input | `_detections_callback` | Entry point; computes `dt` and drives every stage below in order. The only method touching the subscriber. |
  | Association | `_associate` | Greedy nearest-neighbor over a Euclidean cost matrix (`cognitive_tracking.association`), matched against each track's pre-predict position. |
  | Kalman Predict | `_predict` | Coasts every existing track forward by `dt`, matched or not. |
  | Kalman Update | `_update` | Corrects only matched tracks with their associated detection's position. |
  | Lifecycle | `_manage_lifecycle` | Drives each `Track`'s `STATUS_*` state machine and spawns new tentative tracks for unmatched detections. |
  | Assemble | `_build_tracked_array` | Pure message-building, no I/O -- directly unit-testable, same pattern as `perception_node`'s `_build_detection_array`. |
  | Output | `_publish` | The only method that touches the publisher. |

  The Kalman math (`kalman_filter.py`), association (`association.py`), and
  per-track lifecycle (`track.py`) are each their own module with no ROS/rclpy
  imports, so all three are unit-testable with plain numpy arrays / fabricated
  `Track` objects, independent of `tracking_node` and of each other.

- **Association runs against each track's *last known* (pre-predict) position,
  not its predicted-forward position.** The pipeline order is Association ->
  Predict -> Update, not the more common Predict -> Association -> Update. At
  `perception_node`'s 10 Hz publish rate the difference is negligible, and it
  keeps association testable in isolation (no Kalman state has to be advanced
  first to test matching). This is a deliberate Phase-1 simplification traded for
  architectural clarity over tracking accuracy (see Goal below) -- worth
  revisiting if a much lower detection rate or much faster obstacles make it
  matter.

- **Greedy nearest-neighbor, not the Hungarian algorithm.** `association.py`
  exposes `build_cost_matrix()` / `greedy_nearest_neighbor()` as free functions
  over plain numpy arrays (never `Track`/`DetectedObject` types), taking a
  `(cost_matrix, gating_threshold) -> (matches, unmatched_tracks,
  unmatched_detections)` shape. A future optimal-assignment replacement (e.g.
  `scipy.optimize.linear_sum_assignment`) is a new function with the same
  signature -- `tracking_node._associate` is the only call site that would need
  to change. Greedy matching is global (sorted by ascending distance across all
  track/detection pairs), not row-by-row, so processing order can't starve a
  detection that's actually closest to a track considered later.

- **Euclidean gating, not Mahalanobis.** `gating_distance_m` is a flat distance
  threshold, not weighted by each track's covariance. `KalmanFilter6D` still
  maintains the full 6x6 state covariance regardless (needed for
  `TrackedObject.covariance` either way), so Mahalanobis gating is a pure
  `_associate` change later, not a data-model change.

- **NumPy is a new dependency for this workspace** (`python3-numpy` in
  `package.xml`) -- no other package here uses it. Confined entirely to
  `kalman_filter.py` and `association.py`; `track.py` and `tracking_node.py` only
  pass numpy arrays through, they don't need to know it's numpy underneath.

- **Lifecycle thresholds are consecutive-count based, not cumulative.** A miss
  resets `hits` to 0 and a hit resets `misses` to 0 (`Track.register_hit` /
  `register_miss`). With the defaults below, a single miss is enough to push a
  `CONFIRMED` track to `STATUS_OCCLUDED` (`occluded_after_misses: 1`), and it
  takes 5 *consecutive* misses to reach `STATUS_LOST`. A track that recovers
  (gets re-matched while `OCCLUDED`) goes straight back to `CONFIRMED`, not
  through `TENTATIVE` again.

  | Parameter | Default | Meaning |
  |---|---|---|
  | `confirm_after_hits` | 3 | Consecutive hits before `TENTATIVE` -> `CONFIRMED` |
  | `occluded_after_misses` | 1 | Consecutive misses before `CONFIRMED`/`OCCLUDED` -> `OCCLUDED` |
  | `lost_after_misses` | 5 | Consecutive misses before -> `STATUS_LOST` |

- **`STATUS_LOST` tracks are published once, then dropped.** Per
  `TrackedObject.msg`'s own comment ("about to be dropped by the tracker"), a
  track that crosses `lost_after_misses` is still included in that cycle's
  `TrackedObjectArray` with `status: STATUS_LOST`, and is only removed from
  `tracking_node`'s internal track list *after* that array is published
  (`_prune_lost_tracks`, called after `_publish`) -- so no consumer ever sees a
  track vanish from `/tracking/tracks` with no terminal status.

- **`class_id` is taken from the newest matched detection outright.** No voting
  or confidence fusion across a track's history in Phase 1 -- `Track.register_hit`
  just overwrites `class_id`/`size` every time a detection matches. Acceptable
  because Module 3's ground-truth detector never misclassifies; a real detector
  backend would need to add voting here.

- **Publishing is synchronous with detection arrival, not its own timer.**
  Unlike `perception_node` (which decouples publishing from its input rate via a
  fixed-rate timer), `tracking_node` publishes once per incoming
  `DetectedObjectArray` -- Module 3 already publishes at a fixed 10 Hz, so a
  second independent timer here would add complexity without benefit in Phase 1.
  `_predict`/`_update` are still separate methods internally (not one combined
  Kalman step), so introducing a predict-only timer later -- e.g. to keep
  publishing while temporarily losing detections -- only means adding a timer
  callback that calls `_predict` + `_build_tracked_array` + `_publish`, not
  restructuring the Kalman logic itself.

- **No build dependency on `cognitive_perception`.** This package only depends on
  `interfaces` for message types and consumes `/perception/detections` purely as
  a runtime topic, consistent with the dependency-decoupling convention already
  established between `cognitive_perception` and `simulation`.

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces cognitive_perception cognitive_tracking
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
```

## Test

**Automated (pure logic, no Gazebo/ROS graph needed):**
```bash
colcon test --packages-select cognitive_tracking
colcon test-result --verbose
```

**Manual (confirm tracks flow end-to-end):**
```bash
ros2 topic hz /tracking/tracks
ros2 topic echo /tracking/tracks --once
```
You should see one `TrackedObject` per currently-tracked obstacle, each with a
stable `track_id` across cycles, `status` progressing `TENTATIVE` ->
`CONFIRMED` after 3 consecutive detections, and non-zero `velocity` once a
dynamic obstacle has moved for a couple of cycles.

## Goal

This module prioritizes clean, staged architecture over tracking accuracy -- it's
the reference implementation that a future DeepSORT or ByteTrack backend should be
able to replace without `interfaces/TrackedObjectArray`, `/tracking/tracks`, or any
downstream node ever changing.
