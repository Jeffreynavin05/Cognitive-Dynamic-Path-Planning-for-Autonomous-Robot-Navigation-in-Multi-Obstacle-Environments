"""Actuation bridge for Phase 1 and Phase 2 alike.

Relays interfaces/README-free geometry_msgs/Twist commands from
/cmd_vel_nav (dynamic_planner's fixed output, every phase -- see
PROJECT_CONTEXT.md section 5E) onto a deployment-selected output topic:
Gazebo's raw /cmd_vel in Phase 1, the real BeetleBot's existing
/cmd_vel_gate arbitration node in Phase 2. This is the one node in the
entire pipeline allowed to know which environment it is running in
(approved design decision, section 15/16) -- every node upstream of it,
including planner_node, is written to be unaware of the distinction, per
this workspace's digital-twin thesis (section 1).

Staged, mirroring every prior module's convention, but thinner: relaying a
Twist to a Twist has no message transformation, so there is no separate
"assemble" stage -- only input, a safety check, and output.
    _cmd_vel_nav_callback  -- INPUT (drive). The only method touching the
                               /cmd_vel_nav subscriber. Relays immediately
                               (push-driven, minimum latency) and records
                               the arrival time for the watchdog.
    _watchdog_check         -- SAFETY (drive). Timer callback; the only
                               method that fires independent of message
                               arrival. If no /cmd_vel_nav has arrived
                               within cmd_timeout_sec, publishes a zero
                               Twist -- a last-line-of-defense stop for
                               abnormal upstream silence (planner_node
                               crash, network partition), distinct from
                               planner_node's own zero-Twist on normal goal
                               completion/cancellation (see this package's
                               README).
    _publish_command          -- OUTPUT. The only method that touches the
                               output publisher.

No independent velocity clamping is performed here -- dynamic_planner
already enforces its own kinematic limits (PROJECT_CONTEXT.md section 9c);
duplicating that check in this bridge was considered and rejected as
redundant scope creep for a module whose job is bridging, not planning
(approved design decision).
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_VEL_NAV_TOPIC = '/cmd_vel_nav'


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # output_topic is deliberately a parameter, not a module-level
        # constant like every other topic name in this workspace (section
        # 11) -- it is the one deployment-environment selection point in
        # the whole pipeline, not a pipeline-stage-boundary contract.
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('watchdog_check_rate_hz', 10.0)

        gp = self.get_parameter
        output_topic = gp('output_topic').value
        self._cmd_timeout_sec = gp('cmd_timeout_sec').value
        watchdog_check_rate_hz = gp('watchdog_check_rate_hz').value

        # None until the first command arrives -- the watchdog stays
        # silent until there is something to have gone stale.
        self._last_received_time = None
        # Prevents the watchdog from re-publishing a zero Twist every
        # single check once it has already fired; reset by the next real
        # command.
        self._watchdog_has_fired = False

        self._output_pub = self.create_publisher(Twist, output_topic, 10)
        self._cmd_vel_nav_sub = self.create_subscription(
            Twist, CMD_VEL_NAV_TOPIC, self._cmd_vel_nav_callback, 10)
        self._watchdog_timer = self.create_timer(1.0 / watchdog_check_rate_hz, self._watchdog_check)

        self.get_logger().info(
            f'controller_node ready: relaying {CMD_VEL_NAV_TOPIC} -> {output_topic} '
            f'(watchdog timeout {self._cmd_timeout_sec}s).')

    # ---- input: drives the relay, records arrival for the watchdog ------------

    def _cmd_vel_nav_callback(self, msg: Twist) -> None:
        self._last_received_time = self.get_clock().now()
        self._watchdog_has_fired = False
        self._publish_command(msg)

    # ---- safety: fires independent of message arrival -------------------------

    def _watchdog_check(self) -> None:
        if self._last_received_time is None or self._watchdog_has_fired:
            return

        elapsed_sec = (self.get_clock().now() - self._last_received_time).nanoseconds / 1e9
        if elapsed_sec >= self._cmd_timeout_sec:
            self._publish_command(Twist())
            self._watchdog_has_fired = True

    # ---- output -----------------------------------------------------------------

    def _publish_command(self, twist: Twist) -> None:
        self._output_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
