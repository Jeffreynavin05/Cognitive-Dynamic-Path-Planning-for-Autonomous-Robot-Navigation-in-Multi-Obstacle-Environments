# Per-obstacle collision risk, computed from its predicted trajectory against
# the robot's own planned path. Consumed by dynamic_planner to prioritize
# which obstacles to avoid most aggressively. This is the output of the
# cognitive "understand + decide" layer, not a raw sensor measurement.

uint8 THREAT_LOW=0
uint8 THREAT_MEDIUM=1
uint8 THREAT_HIGH=2
uint8 THREAT_CRITICAL=3

uint32 track_id
float32 risk_score              # normalized 0.0-1.0, higher = more dangerous
float32 time_to_collision        # seconds; -1.0 if no collision predicted within horizon
float32 path_intersection_prob   # 0.0-1.0 probability predicted paths intersect
float32 relative_speed           # m/s, closing speed toward the robot
float32 distance_to_robot        # metres, current Euclidean distance
uint8 threat_level
