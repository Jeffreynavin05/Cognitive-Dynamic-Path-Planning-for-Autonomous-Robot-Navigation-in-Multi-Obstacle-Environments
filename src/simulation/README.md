# simulation

Gazebo Harmonic environment for Phase 1: BeetleBot URDF, a walls/corridor/obstacle
arena, and a spawner that randomizes obstacle placement and motion on every run.
**Phase-1-only package** — none of its nodes run on the physical robot; the URDF exists
so `robot_state_publisher` can build the same TF tree in sim as on hardware.

## Why this package exists

This is the "digital twin" boundary. Everything downstream (perception, tracking,
prediction, risk, planning) should be able to run completely unaware of whether it's
talking to Gazebo or the real BeetleBot. That only works if this package exposes the
exact same topic names as the real robot's documented interface (`/scan`, `/imu/data`,
`/pi_camera/image_raw`, `/wheel/odom`) — see [`interfaces` decisions](../interfaces/README.md)
for the same portability principle applied to the pipeline's internal messages.

## Design notes

- **4WD modeled as skid-steer.** The real BeetleBot has 4 independently-driven wheels
  with no steering linkage. `gz-sim-diff-drive-system` accepts multiple `<left_joint>`/
  `<right_joint>` entries, which is exactly that — no custom plugin needed.
- **Wall layout is fixed; obstacle placement is randomized per run.** `worlds/multi_obstacle_arena.sdf`
  hand-authors one static structure (outer walls + a single interior divider with a
  1.6m corridor gap at `x∈[-0.8,0.8], y=2.0`). `simulation_manager` spawns 4 static
  and 5-10 dynamic obstacles at rejection-sampled positions on every launch, so the
  structure is reproducible but the challenge is not — matching the spec's "every run
  should be slightly different."
- **Dynamic obstacles use `gz-sim-velocity-control-system`, not wheeled diff-drive.**
  They're moving props, not vehicles — kinematically setting their velocity avoids
  wheel-orientation/rolling-friction bugs for no loss of realism at this scope.
  ⚠️ I'm reasonably but not fully confident this plugin filename is exact for every
  Harmonic point release — see "If obstacles don't move" below if it doesn't load.
- **Ground truth is exposed, not interpreted.** This package bridges
  `/world/multi_obstacle_arena/pose/info` (every entity's pose, as `tf2_msgs/TFMessage`)
  and stops there. Turning that into `interfaces/DetectedObjectArray` (filtering by
  the `static_obstacle_*` / `dynamic_obstacle_*` name prefixes) is Module 3's job, not
  this package's — keeps the environment and the perception logic decoupled.
- **`/cmd_vel` is the sim's raw diff-drive input**, not `/cmd_vel_nav`. The real robot's
  `/cmd_vel_gate` arbitration node doesn't exist in sim yet; Module 8 (`robot_controller`)
  adds the equivalent relay so Nav2's real output topic works in both places.

## Build & run

```bash
cd cognitive_navigation_ws
colcon build --packages-select interfaces simulation
source install/setup.bash
ros2 launch simulation world.launch.py
```

This starts Gazebo Harmonic, spawns the robot in the south room, bridges its sensors/
cmd_vel/odom/tf, and (a few seconds later, once the world's spawn service is up) spawns
and starts wandering the obstacles.

## Test

**Automated (geometry logic only, no Gazebo needed):**
```bash
colcon test --packages-select simulation
colcon test-result --verbose
```

**Manual (the actual point of this module — go look at it):**
```bash
# in another terminal, after world.launch.py is running
ros2 topic hz /scan /imu/data /pi_camera/image_raw /wheel/odom
ros2 topic echo /world/multi_obstacle_arena/pose/info --once
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive the robot manually
```
Confirm in the Gazebo GUI: the robot sits in the south room, 4 blue static boxes and
6 red cylinders are scattered around, and the cylinders wander continuously without
passing through walls.

**If obstacles spawn but don't move:** run `gz topic -l | grep cmd_vel` to see whether
`/model/dynamic_obstacle_0/cmd_vel` etc. actually exist as gz-transport topics. If not,
`gz-sim-velocity-control-system` likely isn't the exact plugin name/filename shipped
in your Harmonic point release — check `gz sim --versions` and the systems plugin list
in your Gazebo install, and update the plugin block in `_dynamic_obstacle_sdf()`
accordingly. This is the one piece of this module built from documentation recall
rather than a tested install, so it's the most likely spot to need a tweak.
