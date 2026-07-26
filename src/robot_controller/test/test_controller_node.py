"""Unit tests for controller_node's relay/watchdog pipeline. Doesn't require
a live subscriber/publisher or Gazebo -- drives the staged private methods
directly, the same direct-method-call pattern every other node's tests in
this workspace use. Watchdog staleness is simulated by rewinding
node._last_received_time with rclpy.duration.Duration rather than sleeping
in real time, keeping these tests instant and deterministic."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

from robot_controller.controller_node import ControllerNode


def _twist(linear_x=1.0, angular_z=0.5):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist


def _make_stale(node):
    node._last_received_time = node.get_clock().now() - Duration(seconds=node._cmd_timeout_sec + 1.0)


def test_relay_republishes_the_received_command_immediately():
    rclpy.init()
    try:
        node = ControllerNode()
        published = []
        node._output_pub.publish = published.append

        node._cmd_vel_nav_callback(_twist(1.0, 0.5))

        assert len(published) == 1
        assert published[0].linear.x == 1.0
        assert published[0].angular.z == 0.5
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_watchdog_does_nothing_before_any_command_has_been_received():
    rclpy.init()
    try:
        node = ControllerNode()
        published = []
        node._output_pub.publish = published.append

        node._watchdog_check()

        assert published == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_watchdog_does_nothing_while_the_command_is_still_fresh():
    rclpy.init()
    try:
        node = ControllerNode()
        node._cmd_vel_nav_callback(_twist())

        published = []
        node._output_pub.publish = published.append
        node._watchdog_check()

        assert published == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_watchdog_publishes_a_zero_command_once_the_timeout_elapses():
    rclpy.init()
    try:
        node = ControllerNode()
        node._cmd_vel_nav_callback(_twist(1.0, 0.5))
        _make_stale(node)

        published = []
        node._output_pub.publish = published.append
        node._watchdog_check()

        assert len(published) == 1
        assert published[0].linear.x == 0.0
        assert published[0].angular.z == 0.0
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_watchdog_fires_only_once_until_a_new_command_arrives():
    rclpy.init()
    try:
        node = ControllerNode()
        node._cmd_vel_nav_callback(_twist())
        _make_stale(node)

        published = []
        node._output_pub.publish = published.append
        node._watchdog_check()
        node._watchdog_check()

        assert len(published) == 1
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_watchdog_can_fire_again_after_a_fresh_command_then_goes_stale_again():
    rclpy.init()
    try:
        node = ControllerNode()
        node._cmd_vel_nav_callback(_twist())
        _make_stale(node)
        node._watchdog_check()  # first stop

        node._cmd_vel_nav_callback(_twist())  # fresh command resets the watchdog
        _make_stale(node)

        published = []
        node._output_pub.publish = published.append
        node._watchdog_check()

        assert len(published) == 1
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_output_topic_defaults_to_cmd_vel():
    rclpy.init()
    try:
        node = ControllerNode()
        assert node._output_pub.topic_name == '/cmd_vel'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_output_topic_is_configurable_via_parameter():
    rclpy.init(args=['--ros-args', '-p', 'output_topic:=/cmd_vel_gate'])
    try:
        node = ControllerNode()
        assert node._output_pub.topic_name == '/cmd_vel_gate'
        node.destroy_node()
    finally:
        rclpy.shutdown()
