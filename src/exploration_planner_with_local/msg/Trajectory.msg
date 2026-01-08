# Smooth trajectory for following
std_msgs/Header header

# Trajectory points (position + yaw)
geometry_msgs/PoseStamped[] poses

# Corresponding velocities at each point
geometry_msgs/Twist[] velocities

# Time stamps for each point (relative to header.stamp)
float64[] time_from_start

# Total trajectory duration
float64 total_time