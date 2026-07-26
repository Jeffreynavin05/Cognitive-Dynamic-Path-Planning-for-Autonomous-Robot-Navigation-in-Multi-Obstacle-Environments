# A single object detected in one sensor frame, before tracking/ID assignment.
# Produced by perception_node. Any detector backend (ground-truth, YOLO, LiDAR
# clustering) must publish this exact shape so cognitive_tracking never changes.

uint8 CLASS_UNKNOWN=0
uint8 CLASS_PEDESTRIAN=1
uint8 CLASS_DYNAMIC_OBSTACLE=2
uint8 CLASS_STATIC_OBSTACLE=3

std_msgs/Header header          # frame_id is the world frame the position is in (e.g. "odom")
uint8 class_id
geometry_msgs/Point position     # object centroid
geometry_msgs/Vector3 size        # bounding box extent (x=length, y=width, z=height), metres
float32 confidence                # detector confidence, 0.0-1.0 (1.0 for ground truth)
