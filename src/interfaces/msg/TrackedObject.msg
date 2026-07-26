# A tracked object with a persistent ID and estimated kinematics.
# Produced by cognitive_tracking. Any tracker backend (this project's Kalman
# tracker, DeepSORT, ByteTrack) must publish this exact shape so
# motion_prediction never changes.

uint8 STATUS_TENTATIVE=0   # seen too few frames to be confirmed
uint8 STATUS_CONFIRMED=1   # stable, high-confidence track
uint8 STATUS_OCCLUDED=2    # not currently detected, position coasted from last known state
uint8 STATUS_LOST=3        # about to be dropped by the tracker

std_msgs/Header header
uint32 track_id
uint8 class_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
float64[36] covariance     # 6x6 state covariance for [x,y,z,vx,vy,vz], row-major
uint8 status
uint32 age                 # consecutive frames this ID has been tracked
