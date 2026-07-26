"""Unit tests for camera_node/lidar_node's placeholder heartbeat counters.
Doesn't touch real sensor topics -- calls the callback/timer methods directly,
same pattern as test_perception_node.py."""
import rclpy
from sensor_msgs.msg import Image, LaserScan

from cognitive_perception.camera_node import CameraNode
from cognitive_perception.lidar_node import LidarNode


def test_camera_node_counts_frames_and_resets_on_heartbeat():
    rclpy.init()
    try:
        node = CameraNode()
        for _ in range(3):
            node._image_callback(Image())
        assert node._frame_count == 3

        node._log_heartbeat()
        assert node._frame_count == 0

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_lidar_node_counts_scans_and_resets_on_heartbeat():
    rclpy.init()
    try:
        node = LidarNode()
        for _ in range(5):
            node._scan_callback(LaserScan())
        assert node._scan_count == 5

        node._log_heartbeat()
        assert node._scan_count == 0

        node.destroy_node()
    finally:
        rclpy.shutdown()
