"""Spawns and randomly wanders the static/dynamic obstacles for the multi-obstacle arena.

Owns the *environment*, not the robot: obstacle placement, obstacle motion, and nothing
else. Ground-truth pose extraction for the perception pipeline is deliberately NOT this
node's job -- it publishes nothing about obstacle identity/class itself. Each spawned
obstacle carries its own PosePublisher plugin (see OBSTACLE_POSE_PUBLISHER_PLUGIN and
*_pose_topic() below) so cognitive_perception (Module 3) can read named, per-entity
transforms off the bridged per-model topics and turn them into
interfaces/DetectedObjectArray. A plain /world/<world>/pose/info bridge can't be used
for this: SceneBroadcaster's Pose_V never sets the header data (frame_id/
child_frame_id) that the tf2_msgs ros_gz_bridge conversion reads, so every
child_frame_id comes through empty -- only PosePublisher fills it in.

Confirmed on a real Harmonic (gz-sim 8) install: this build's PosePublisher does *not*
honor a custom <topic> override on a per-model plugin (silently keeps publishing on its
default topic instead, with no error) -- so *_pose_topic() below mirror the verified
defaults instead of trying to consolidate onto one shared topic: `/model/<name>/pose`
for a normal (moving) publisher, `/model/<name>/pose_static` for a `static_publisher`
one (gz-sim never publishes a static_publisher entity on the plain, non-`_static`
topic at all, since a truly static model never emits the pose-changed event the normal
path relies on). world.launch.py's bridge args and perception_node's subscriptions
both re-derive these same topic strings from the matching obstacle names.
"""
import math
import random
import subprocess
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile


def static_obstacle_pose_topic(name: str) -> str:
    return f'/model/{name}/pose_static'


def dynamic_obstacle_pose_topic(name: str) -> str:
    return f'/model/{name}/pose'


OBSTACLE_POSE_PUBLISHER_PLUGIN = '''<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>{static_publisher}</static_publisher>
      <static_update_frequency>{update_frequency}</static_update_frequency>
      <update_frequency>{update_frequency}</update_frequency>
    </plugin>'''


@dataclass
class DynamicObstacleState:
    name: str
    x: float
    y: float
    target_x: float
    target_y: float
    speed: float


class SimulationManager(Node):

    def __init__(self):
        super().__init__('simulation_manager')

        self.declare_parameter('world_name', 'multi_obstacle_arena')
        self.declare_parameter('arena_x_min', -6.5)
        self.declare_parameter('arena_x_max', 6.5)
        self.declare_parameter('arena_y_min', -4.5)
        self.declare_parameter('arena_y_max', 4.5)
        self.declare_parameter('corridor_band_y_min', 1.5)
        self.declare_parameter('corridor_band_y_max', 2.5)
        self.declare_parameter('corridor_half_width', 1.0)
        self.declare_parameter('robot_spawn_x', -5.0)
        self.declare_parameter('robot_spawn_y', -3.5)
        self.declare_parameter('robot_keepout_radius', 1.2)
        self.declare_parameter('num_static_obstacles', 4)
        self.declare_parameter('static_obstacle_min_size', 0.3)
        self.declare_parameter('static_obstacle_max_size', 0.6)
        self.declare_parameter('num_dynamic_obstacles', 6)
        self.declare_parameter('dynamic_obstacle_radius', 0.2)
        self.declare_parameter('min_obstacle_speed', 0.15)
        self.declare_parameter('max_obstacle_speed', 0.5)
        self.declare_parameter('waypoint_arrival_radius', 0.3)
        self.declare_parameter('min_entity_separation', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('random_seed', 0)

        gp = self.get_parameter
        self.world_name = gp('world_name').value
        self.x_min = gp('arena_x_min').value
        self.x_max = gp('arena_x_max').value
        self.y_min = gp('arena_y_min').value
        self.y_max = gp('arena_y_max').value
        self.corridor_y_min = gp('corridor_band_y_min').value
        self.corridor_y_max = gp('corridor_band_y_max').value
        self.corridor_half_width = gp('corridor_half_width').value
        self.robot_x = gp('robot_spawn_x').value
        self.robot_y = gp('robot_spawn_y').value
        self.robot_keepout = gp('robot_keepout_radius').value
        self.num_static = gp('num_static_obstacles').value
        self.static_min_size = gp('static_obstacle_min_size').value
        self.static_max_size = gp('static_obstacle_max_size').value
        self.num_dynamic = gp('num_dynamic_obstacles').value
        self.dynamic_radius = gp('dynamic_obstacle_radius').value
        self.min_speed = gp('min_obstacle_speed').value
        self.max_speed = gp('max_obstacle_speed').value
        self.arrival_radius = gp('waypoint_arrival_radius').value
        self.min_sep = gp('min_entity_separation').value
        self.control_rate = gp('control_rate_hz').value
        seed = gp('random_seed').value

        self._rng = random.Random(seed if seed else None)
        self._occupied: list[tuple[float, float]] = [(self.robot_x, self.robot_y)]
        self._dynamic_obstacles: list[DynamicObstacleState] = []
        self._cmd_publishers = {}
        self._timer = None

        if not 5 <= self.num_dynamic <= 10:
            self.get_logger().warn(
                f'num_dynamic_obstacles={self.num_dynamic} is outside the project spec '
                'range of 5-10; continuing anyway.')

    def start(self) -> None:
        """Spawn obstacles and begin the wander control loop. Not called from __init__
        so this node can be constructed and unit-tested without touching Gazebo."""
        self._spawn_static_obstacles()
        self._spawn_dynamic_obstacles()
        self._timer = self.create_timer(1.0 / self.control_rate, self._control_step)
        self.get_logger().info(
            f'simulation_manager ready: {self.num_static} static + '
            f'{self.num_dynamic} dynamic obstacles in world "{self.world_name}".')

    # ---- spatial sampling ---------------------------------------------------

    def _in_wall_band(self, x: float, y: float) -> bool:
        """True if (x, y) falls inside the corridor divider walls (see world SDF)."""
        return (self.corridor_y_min <= y <= self.corridor_y_max
                and abs(x) >= self.corridor_half_width)

    def _in_robot_keepout(self, x: float, y: float) -> bool:
        return math.hypot(x - self.robot_x, y - self.robot_y) < self.robot_keepout

    def _far_enough(self, x: float, y: float) -> bool:
        return all(math.hypot(x - ox, y - oy) >= self.min_sep for ox, oy in self._occupied)

    def _sample_free_xy(self, max_attempts: int = 200) -> tuple[float, float]:
        for _ in range(max_attempts):
            x = self._rng.uniform(self.x_min, self.x_max)
            y = self._rng.uniform(self.y_min, self.y_max)
            if not self._in_wall_band(x, y) and not self._in_robot_keepout(x, y) \
                    and self._far_enough(x, y):
                return x, y
        self.get_logger().warn(
            f'Could not find a fully free spawn point after {max_attempts} attempts; '
            'using the last sampled point (may be closely packed).')
        return x, y

    def _pick_new_waypoint(self) -> tuple[float, float]:
        for _ in range(50):
            x = self._rng.uniform(self.x_min, self.x_max)
            y = self._rng.uniform(self.y_min, self.y_max)
            if not self._in_wall_band(x, y):
                return x, y
        return 0.0, 0.0

    # ---- spawning -------------------------------------------------------------

    def _spawn_entity(self, name: str, sdf: str, x: float, y: float, yaw: float = 0.0) -> None:
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-name', name,
            '-string', sdf,
            '-x', str(x), '-y', str(y), '-z', '0.05', '-Y', str(yaw),
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'Failed to spawn {name}: {result.stderr.strip()}')
        else:
            self.get_logger().info(f'Spawned {name} at ({x:.2f}, {y:.2f})')

    def _static_obstacle_sdf(self, name: str, size: float) -> str:
        return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material><ambient>0.2 0.4 0.8 1</ambient><diffuse>0.2 0.4 0.8 1</diffuse></material>
      </visual>
    </link>
    {OBSTACLE_POSE_PUBLISHER_PLUGIN.format(static_publisher='true', update_frequency='1')}
  </model>
</sdf>'''

    def _spawn_static_obstacles(self) -> None:
        for i in range(self.num_static):
            x, y = self._sample_free_xy()
            self._occupied.append((x, y))
            size = self._rng.uniform(self.static_min_size, self.static_max_size)
            name = f'static_obstacle_{i}'
            self._spawn_entity(name, self._static_obstacle_sdf(name, size), x, y)

    def _dynamic_obstacle_sdf(self, name: str) -> str:
        # Kinematic VelocityControl instead of wheel joints: these are moving props,
        # not vehicles, and don't need wheel-rolling physics to look and behave right.
        r = self.dynamic_radius
        return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>{r}</radius><length>0.3</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{r}</radius><length>0.3</length></cylinder></geometry>
        <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <topic>model/{name}/cmd_vel</topic>
    </plugin>
    {OBSTACLE_POSE_PUBLISHER_PLUGIN.format(static_publisher='false', update_frequency=str(self.control_rate))}
  </model>
</sdf>'''

    def _spawn_dynamic_obstacles(self) -> None:
        for i in range(self.num_dynamic):
            x, y = self._sample_free_xy()
            self._occupied.append((x, y))
            tx, ty = self._pick_new_waypoint()
            speed = self._rng.uniform(self.min_speed, self.max_speed)
            name = f'dynamic_obstacle_{i}'
            self._spawn_entity(name, self._dynamic_obstacle_sdf(name), x, y)
            self._dynamic_obstacles.append(
                DynamicObstacleState(name=name, x=x, y=y, target_x=tx, target_y=ty, speed=speed))
            self._cmd_publishers[name] = self.create_publisher(
                Twist, f'/model/{name}/cmd_vel', QoSProfile(depth=10))

    # ---- control loop -----------------------------------------------------------

    def _control_step(self) -> None:
        step = 1.0 / self.control_rate
        for obs in self._dynamic_obstacles:
            dx = obs.target_x - obs.x
            dy = obs.target_y - obs.y
            dist = math.hypot(dx, dy)

            if dist < self.arrival_radius:
                obs.target_x, obs.target_y = self._pick_new_waypoint()
                continue

            heading = math.atan2(dy, dx)
            twist = Twist()
            # Obstacles never yaw (angular.z stays 0), so their body frame never
            # diverges from world frame -- world-frame vx/vy here is equivalent to
            # VelocityControl's body-frame linear.x/y semantics for a non-rotating body.
            twist.linear.x = obs.speed * math.cos(heading)
            twist.linear.y = obs.speed * math.sin(heading)
            self._cmd_publishers[obs.name].publish(twist)

            obs.x += twist.linear.x * step
            obs.y += twist.linear.y * step


def main(args=None):
    rclpy.init(args=args)
    node = SimulationManager()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
