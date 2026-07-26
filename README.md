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
| `motion_prediction` | planned | Future trajectory forecasting |
| `risk_assessment` | planned | Per-obstacle collision risk scoring (TTC, path intersection, etc.) |
| `dynamic_planner` | planned | Nav2 integration, custom risk-aware costmap layer, MPPI controller |
| `robot_controller` | planned | Bridges planner output to `/cmd_vel_nav`, safety envelope |
| `cognitive_bringup` | planned | Launch files, RViz configs, world files, visualization_node |

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
