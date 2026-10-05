# Cartesian vectors expressed in header.frame_id (inertial, z up).
std_msgs/Header header
# Changes whenever the simulation is reset or a new attempt starts.
uint64 run_id
geometry_msgs/Point position_m
geometry_msgs/Vector3 velocity_mps
geometry_msgs/Vector3 acceleration_mps2
# Rotation from the interceptor body frame to header.frame_id.
geometry_msgs/Quaternion attitude
