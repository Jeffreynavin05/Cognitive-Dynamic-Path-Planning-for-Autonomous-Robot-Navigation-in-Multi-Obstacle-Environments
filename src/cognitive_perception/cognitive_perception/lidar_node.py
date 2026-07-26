"""Phase-2 placeholder for the real LiDAR-based detector.

Subscribes to the bridged RPLidar topic and logs a periodic scan-rate heartbeat
only -- it does not run clustering and does not publish interfaces/DetectedObjectArray.
Mirrors camera_node; see that file's docstring and cognitive_perception/README.md for
why this is a thin stub in Phase 1 rather than real LiDAR clustering logic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('heartbeat_period_sec', 5.0)

        gp = self.get_parameter
        self.scan_topic = gp('scan_topic').value
        self.heartbeat_period = gp('heartbeat_period_sec').value

        self._scan_count = 0
        self._sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self._timer = self.create_timer(self.heartbeat_period, self._log_heartbeat)

        self.get_logger().info(
            f'lidar_node ready (Phase-2 placeholder): listening on "{self.scan_topic}", '
            'no detections published -- perception_node uses ground truth in Phase 1.')

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_count += 1

    def _log_heartbeat(self) -> None:
        rate = self._scan_count / self.heartbeat_period
        self.get_logger().info(
            f'lidar_node heartbeat: ~{rate:.1f} Hz ({self._scan_count} scans since last check)')
        self._scan_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
