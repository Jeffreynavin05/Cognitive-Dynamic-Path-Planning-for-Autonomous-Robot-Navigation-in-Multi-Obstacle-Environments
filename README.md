# cognitive_navigation_ws

ROS2 Jazzy workspace for **Cognitive Dynamic Path Planning for Autonomous Robot
Navigation in Multi-Obstacle Environments** — a final-year B.Tech project. Built
and validated in Gazebo Harmonic (Phase 1), then deployed unmodified onto the
physical BeetleBot (Phase 2).

## Pipeline

```
camera_node ─┐
lidar_node ──┴─→ perception_node → tracking_node → prediction_node → risk_node → planner_node → controller_node
                                          │               │              │            │
                                          └───────────────┴──────────────┴────────────┴──→ visualization_node
```

Every arrow above is a fixed message contract defined in `interfaces/` — see that
package's README for the full message table and the reasoning behind it. The
contract is what lets each stage's implementation be replaced later (ground-truth
detection → YOLO, Kalman tracker → DeepSORT/ByteTrack, constant-velocity predictor →
LSTM) without touching any other node.

## Packages

| Package | Status | Responsibility |
|---|---|---|
| `interfaces` | done | Shared message definitions |
| `simulation` | done | Gazebo Harmonic world, URDF, randomized moving-obstacle spawner |
| `cognitive_perception` | done | camera_node, lidar_node, perception_node |
| `cognitive_tracking` | done | Multi-object tracking with persistent IDs |
| `motion_prediction` | done | Constant-velocity trajectory forecasting per tracked object |
| `risk_assessment` | done | Per-obstacle collision risk scoring (TTC, path intersection, etc.) |
| `dynamic_planner` | done | Self-hosted `NavigateToPose` action server; deterministic, risk-aware local planner |
| `robot_controller` | done | Relays `/cmd_vel_nav` onto `/cmd_vel` (sim) / `/cmd_vel_gate` (hardware), with a command-staleness watchdog |
| `cognitive_bringup` | done | One-command launch files, RViz config, visualization_node |

The full Phase-1 pipeline (`perception_node` through `controller_node`) is complete and runs
end-to-end. See `PROJECT_CONTEXT.md` for full architectural detail, per-module implementation
notes, and the design decisions behind each stage.

## Quick start (Phase 1, Gazebo)

```bash
ros2 launch cognitive_bringup bringup_sim.launch.py
```

Starts Gazebo, spawns the robot and every obstacle, and launches the full pipeline plus
RViz. See `src/cognitive_bringup/README.md` for sending a goal and the Phase-2 launch stub.

## Target platform

BeetleBot: Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy), RPLidar C1, Pi Camera V1.3,
LSM6DSRTR IMU, 4WD differential-style drive via STM32 "Lyra" motor controller.
Nav2 output must publish to `/cmd_vel_nav` (not `/cmd_vel`) so it passes through the
robot's existing `/cmd_vel_gate` arbitration node on real hardware.

## Build

Requires ROS2 Jazzy on Ubuntu 24.04 (or a container/VM providing it — this repo
is authored from macOS, which cannot build ROS2 packages natively).

```bash
cd cognitive_navigation_ws
colcon build
source install/setup.bash
```
