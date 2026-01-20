std_msgs/Header header

# Velocity
float64 velocity_magnitude      # [m/s] Anlık hız
float64 velocity_horizontal     # [m/s] Yatay hız
float64 average_velocity        # [m/s] Ortalama hız

# Acceleration
float64 acceleration_magnitude  # [m/s²] Anlık ivme

# Yaw
float64 current_yaw             # [rad] Mevcut yaw
float64 target_yaw              # [rad] Hedef yaw
float64 yaw_error               # [rad] Yaw farkı

# Target Viewpoint
geometry_msgs/Point target_position
float64 target_coverage
uint32 target_cluster_id

# Progress
uint32 current_waypoint_index
uint32 total_waypoints
float64 distance_to_goal

# Path Statistics
float64 total_path_traveled     # [m] Toplam alınan yol
float64 session_time            # [s] Keşif süresi
