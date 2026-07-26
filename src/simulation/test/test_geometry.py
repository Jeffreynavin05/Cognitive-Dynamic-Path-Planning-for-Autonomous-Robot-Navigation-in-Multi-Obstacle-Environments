"""Unit tests for simulation_manager's spatial sampling logic. Doesn't touch Gazebo --
SimulationManager.start() (which spawns entities) is deliberately not called here."""
import rclpy

from simulation.simulation_manager import SimulationManager


def test_wall_band_and_free_sampling():
    rclpy.init()
    try:
        node = SimulationManager()

        # Points known to fall inside the corridor divider walls (outside the 1.6m gap).
        assert node._in_wall_band(x=2.0, y=2.0) is True
        assert node._in_wall_band(x=-2.0, y=2.0) is True

        # Points inside the corridor gap, or outside the wall's y-band entirely, are clear.
        assert node._in_wall_band(x=0.0, y=2.0) is False
        assert node._in_wall_band(x=2.0, y=0.0) is False

        # Robot keep-out circle.
        assert node._in_robot_keepout(node.robot_x, node.robot_y) is True
        assert node._in_robot_keepout(node.robot_x + 5.0, node.robot_y) is False

        # The free-space sampler must never return a point inside the wall band or
        # the robot keep-out circle.
        for _ in range(20):
            x, y = node._sample_free_xy()
            assert not node._in_wall_band(x, y)
            assert not node._in_robot_keepout(x, y)

        node.destroy_node()
    finally:
        rclpy.shutdown()
