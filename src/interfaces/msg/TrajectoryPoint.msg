# One predicted future state along an obstacle's forecast trajectory.
builtin_interfaces/Time stamp     # absolute predicted time
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[9] covariance              # 3x3 position covariance, row-major (x,y,z); grows with horizon
