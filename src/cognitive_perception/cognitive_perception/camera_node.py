"""Phase-2 placeholder for the real camera-based detector.

Subscribes to the bridged Pi Camera topic and logs a periodic frame-rate heartbeat
only -- it does not run inference and does not publish interfaces/DetectedObjectArray.
It exists now so the node name, executable entry, and topic wiring already match the
top-level README's pipeline diagram (camera_node -> perception_node -> ...) before
Phase 2 needs to drop real detection logic in here. perception_node does not depend
on this node in Phase 1; it reads Gazebo ground truth directly instead.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('image_topic', '/pi_camera/image_raw')
        self.declare_parameter('heartbeat_period_sec', 5.0)

        gp = self.get_parameter
        self.image_topic = gp('image_topic').value
        self.heartbeat_period = gp('heartbeat_period_sec').value

        self._frame_count = 0
        self._sub = self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self._timer = self.create_timer(self.heartbeat_period, self._log_heartbeat)

        self.get_logger().info(
            f'camera_node ready (Phase-2 placeholder): listening on "{self.image_topic}", '
            'no detections published -- perception_node uses ground truth in Phase 1.')

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1

    def _log_heartbeat(self) -> None:
        rate = self._frame_count / self.heartbeat_period
        self.get_logger().info(
            f'camera_node heartbeat: ~{rate:.1f} Hz ({self._frame_count} frames since last check)')
        self._frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
