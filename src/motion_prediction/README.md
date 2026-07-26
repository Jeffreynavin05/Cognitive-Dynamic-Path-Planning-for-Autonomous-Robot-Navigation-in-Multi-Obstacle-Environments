# motion_prediction

Module 5 of the cognitive navigation pipeline: turns `interfaces/TrackedObjectArray`
(persistent, ID-stable, Kalman-smoothed tracks from `cognitive_tracking`) into
`interfaces/PredictedTrajectoryArray` -- a forecast trajectory per tracked object --
the fixed contract `risk_assessment` (Module 6) and `visualization_node` consume.

## Why this package exists

Per the top-level README's pipeline (`tracking_node` -> `prediction_node` ->
`risk_node` -> ...), this package owns everything between a smoothed track estimate
and a forward-looking motion forecast. Keeping the predictor *implementation*
separate from the `interfaces/PredictedTrajectoryArray` *contract* is what lets
this project's constant-velocity forecaster be swapped for an LSTM or transformer
model later without `risk_assessment` (or anything downstream) changing at all --
see `interfaces/msg/PredictedTrajectory.msg`'s own header comment.

## Node

| Node | Role |
|---|---|
| `prediction_node` | Subscribes to `/tracking/tracks`, publishes `PredictedTrajectoryArray` on `/prediction/trajectories` synchronously, once per incoming `TrackedObjectArray` |

## Design notes

- **`prediction_node` is staged into distinct methods**, mirroring
  `cognitive_perception`/`cognitive_tracking`'s convention so a future backend swap
  only ever touches one stage:

  | Stage | Method | Job |
  |---|---|---|
  | Input | `_tracks_callback` | Entry point; drives every stage below in order. The only method touching the subscriber. |
  | Filter | `_select_predictable_tracks` | Keeps only `STATUS_CONFIRMED`/`STATUS_OCCLUDED` tracks -- see below. |
  | Predict | `_predict_trajectory` | Per-track constant-velocity forward projection (`motion_prediction.trajectory_predictor`). |
  | Assemble | `_build_trajectory_array` | Pure message-building, no I/O -- directly unit-testable. |
  | Output | `_publish` | The only method that touches the publisher. |

  The forecast math (`trajectory_predictor.py`) is its own module with no ROS/rclpy
  imports, unit-testable with plain numpy arrays, independent of `prediction_node`.

- **Only `STATUS_CONFIRMED` and `STATUS_OCCLUDED` tracks are forecast.**
  `STATUS_TENTATIVE` tracks have too little history to trust (as few as one hit),
  and `STATUS_LOST` tracks are already about to be dropped by `tracking_node` --
  forecasting either would be forecasting noise. `STATUS_OCCLUDED` tracks are
  included deliberately: their position/velocity are still a coasted Kalman
  estimate worth projecting forward while the tracker waits to reconfirm or drop
  them.

- **Constant-velocity forecast, reimplemented locally -- not imported from
  `cognitive_tracking`.** `trajectory_predictor.py`'s `propagate_state()`
  reconstructs the same state-transition (`F`) and process-noise (`Q`) matrices as
  `cognitive_tracking.kalman_filter.KalmanFilter6D.predict()`, applied
  `horizon_sec / step_sec` times in sequence starting from each `TrackedObject`'s
  published position, velocity, and 6x6 covariance. The math is deliberately
  equivalent (a forecast should grow uncertainty the same way the tracker's own
  filter does), but the code is **not** shared or imported across the package
  boundary -- per this workspace's rule that no pipeline-stage package depends on
  another stage package for anything but its published topic (`PROJECT_CONTEXT.md`
  §7/§16). A future accuracy improvement to one filter's `Q` construction does not
  silently change the other's forecast.

- **`process_noise_std` is an independent parameter from `cognitive_tracking`'s.**
  Declared in this package's own `config/prediction_params.yaml`, not read from or
  synced with `cognitive_tracking/config/tracking_params.yaml` -- consistent with
  this workspace's existing convention of deliberately duplicating parameters
  across a package boundary rather than sharing them (see `num_static_obstacles` in
  `PROJECT_CONTEXT.md` §13).

- **Publishing is synchronous with track arrival, not its own timer** -- same
  reasoning as `tracking_node`: `cognitive_tracking` already publishes at a fixed
  cadence, so a second independent timer here would add complexity without benefit
  in Phase 1.

- **`model_name` is hardcoded to `"constant_velocity"`.** Per
  `interfaces/msg/PredictedTrajectory.msg`'s own comment, this field exists purely
  so a future backend (e.g. `"lstm_v1"`) identifies itself for logging/debugging --
  `risk_assessment` must never branch on it.

- **No build dependency on `cognitive_tracking`.** This package only depends on
  `interfaces` for message types and consumes `/tracking/tracks` purely as a
  runtime topic, consistent with the dependency-decoupling convention already
  established between `cognitive_tracking` and `cognitive_perception`.

## Parameters (`config/prediction_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `horizon_sec` | 3.0 | How far into the future to forecast each track, in seconds |
| `step_sec` | 0.1 | Spacing between forecast points, in seconds (30 points/track at the defaults) |
| `process_noise_std` | 0.5 | Process-noise std for this package's own covariance propagation -- independent of `cognitive_tracking`'s value of the same name |

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces cognitive_tracking motion_prediction
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
```

## Test

**Automated (pure logic, no Gazebo/ROS graph needed):**
```bash
colcon test --packages-select motion_prediction
colcon test-result --verbose
```

**Manual (confirm trajectories flow end-to-end):**
```bash
ros2 topic hz /prediction/trajectories
ros2 topic echo /prediction/trajectories --once
```
You should see one `PredictedTrajectory` per currently `CONFIRMED`/`OCCLUDED`
track, each with `model_name: constant_velocity` and 30 `TrajectoryPoint`s (at the
default 3.0s horizon / 0.1s step) whose position covariance grows across the
array.

## Goal

This module prioritizes clean, staged architecture over forecasting accuracy --
it's the reference implementation that a future LSTM or transformer-based
predictor should be able to replace (by rewriting only `trajectory_predictor.py`)
without `interfaces/PredictedTrajectoryArray`, `/prediction/trajectories`, or any
downstream node ever changing.
