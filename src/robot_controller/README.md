# robot_controller

Module 8 of the cognitive navigation pipeline, and the final Phase-1 module: bridges
`dynamic_planner`'s `/cmd_vel_nav` output onto real actuation — Gazebo's raw `/cmd_vel`
input in simulation, the real BeetleBot's existing `/cmd_vel_gate` arbitration node on
hardware.

## Why this package exists

Per this workspace's digital-twin thesis (`PROJECT_CONTEXT.md` section 1), every node from
perception through planning is written to be unaware of whether it's talking to Gazebo or the
physical robot. That only works if *something* absorbs the one unavoidable difference between
the two environments — Gazebo takes raw velocity commands directly, the real robot arbitrates
between multiple command sources first. `robot_controller` is that one node, deliberately kept
as small and late in the pipeline as possible so the environment-awareness doesn't leak
upstream.

## Node

| Node | Role |
|---|---|
| `controller_node` | Subscribes to `/cmd_vel_nav` (`geometry_msgs/Twist`), republishes immediately onto a deployment-selected `output_topic`, and runs a command-staleness watchdog |

## Design notes

- **`output_topic` is a parameter, not a fixed constant — the one deliberate exception to
  this workspace's "topic names are never ROS parameters" convention** (`PROJECT_CONTEXT.md`
  section 11). Every other pipeline-contract topic name is a module-level constant because
  changing it would silently break a neighboring stage's expectations; `output_topic` is
  different in kind — it's not a pipeline-stage boundary, it's the sim/hardware bridge
  selection point (the same category as `/scan`, `/wheel/odom`, etc. in section 5A), and it's
  *meant* to be reconfigured per deployment. Default `/cmd_vel` (Phase 1); override to
  `/cmd_vel_gate` for Phase 2. No in-code branching — `controller_node`'s relay logic is
  identical either way, it just publishes somewhere else. Approved design decision.

- **Command-staleness watchdog, not a pure relay.** `planner_node` already publishes an
  explicit zero `Twist` on normal `NavigateToPose` success/cancellation (`PROJECT_CONTEXT.md`
  section 9c), but that only covers the well-behaved case. If `/cmd_vel_nav` goes silent
  abnormally mid-motion (a `planner_node` crash, a network partition), the last nonzero command
  would otherwise keep driving the robot indefinitely — most Gazebo diff-drive plugins (and
  plausibly a real motor controller) apply the most recently received `Twist` continuously
  until superseded, not just once. `_watchdog_check` runs on its own timer
  (`watchdog_check_rate_hz`) and publishes a zero `Twist` if no command has arrived within
  `cmd_timeout_sec`, firing only once per stale period (reset by the next real command) so it
  doesn't spam redundant stops. A distinctly-placed last-line-of-defense safety measure
  appropriate for the actuation bridge specifically. Approved design decision.

- **No independent velocity clamping.** `dynamic_planner` already enforces its own kinematic
  limits (`max_linear_speed_mps`/`max_angular_speed_radps`, `PROJECT_CONTEXT.md` section 9c).
  Duplicating that check here was considered and rejected — this module's job is bridging, not
  planning, and a second place to keep velocity limits in sync would be redundant scope creep
  rather than genuine defense-in-depth. Approved design decision.

- **No `assemble` stage.** Every other node in this workspace separates a pure "assemble"
  method (build a message) from an "output" method (touch a publisher) — see section 10's
  staged-method convention. Relaying a `Twist` to a `Twist` has no message transformation at
  all, so there is nothing to assemble; `_publish_command` is the only output-touching method,
  and it's called directly with the message as received (or a fresh zero `Twist` from the
  watchdog).

- **Staged design**, thinner than every prior module since there's no filter/predict/join/plan
  stage here, only a safety check:

  | Stage | Method | Job |
  |---|---|---|
  | Input | `_cmd_vel_nav_callback` | Relays immediately (push-driven, minimum latency); records arrival time for the watchdog |
  | Safety | `_watchdog_check` | Timer callback; publishes a zero `Twist` if `/cmd_vel_nav` has gone stale |
  | Output | `_publish_command` | The only method that touches the output publisher |

- **Plain `rclpy.spin()`, not `MultiThreadedExecutor`.** Unlike `dynamic_planner`'s
  `planner_node`, nothing here blocks — the subscription callback and the watchdog timer are
  both quick, non-blocking calls, so the default single-threaded executor handles both without
  risk of deadlock (`PROJECT_CONTEXT.md` section 11's `MultiThreadedExecutor` exception does
  not apply to this module).

- **No dependency on `interfaces`.** This is the first pipeline-stage package with no use for
  any custom message type at all — `geometry_msgs/Twist` in, `geometry_msgs/Twist` out, no
  transformation. No dependency on `dynamic_planner` either; `/cmd_vel_nav` is consumed purely
  as a runtime topic, consistent with the dependency-decoupling convention established across
  every earlier stage.

## Parameters (`config/controller_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `output_topic` | `/cmd_vel` | Deployment-selected relay destination — override to `/cmd_vel_gate` for Phase 2 |
| `cmd_timeout_sec` | 0.5 | Watchdog: publish a zero `Twist` if `/cmd_vel_nav` has been silent this long |
| `watchdog_check_rate_hz` | 10.0 | How often the watchdog checks for staleness |

## Build

```bash
cd cognitive_navigation_ws
colcon build --packages-select dynamic_planner robot_controller
source install/setup.bash
```

## Run

```bash
# terminal 1
ros2 launch simulation world.launch.py

# terminals 2-6, once Gazebo and the obstacles have spawned
ros2 launch cognitive_perception perception.launch.py
ros2 launch cognitive_tracking tracking.launch.py
ros2 launch motion_prediction prediction.launch.py
ros2 launch risk_assessment risk.launch.py
ros2 launch dynamic_planner planner.launch.py

# terminal 7
ros2 launch robot_controller controller.launch.py

# terminal 8, send a goal -- the robot should now actually move in Gazebo
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}}}}"
```

For Phase 2 deployment onto the real BeetleBot, launch with
`ros2 launch robot_controller controller.launch.py output_topic:=/cmd_vel_gate` (or set it in
`config/controller_params.yaml`) — no other package changes.

## Test

**Automated (pure logic, no Gazebo/ROS graph needed):**
```bash
colcon test --packages-select robot_controller
colcon test-result --verbose
```

**Manual (confirm the full pipeline finally moves the robot):**
```bash
ros2 topic echo /cmd_vel
```
After sending a goal (with the full pipeline running), you should see the same commands
`planner_node` publishes on `/cmd_vel_nav` appear on `/cmd_vel`, and Gazebo's BeetleBot should
actually move. Kill `planner_node` mid-motion and confirm `/cmd_vel` goes to zero within
`cmd_timeout_sec`.

## Goal

This is the closing module of the Phase-1 reference pipeline — deliberately the thinnest,
most mechanical node in the workspace, so that swapping in a real `/cmd_vel_gate` arbitration
behavior later (or a completely different actuation bridge) never requires touching
`dynamic_planner` or anything upstream of it.
