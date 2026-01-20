# Exploration tour - sequence of waypoints to visit
std_msgs/Header header

# Current exploration state
uint8 state
uint8 IDLE = 0
uint8 EXPLORING = 1
uint8 MOVING_TO_VIEWPOINT = 2
uint8 REFINING = 3
uint8 COMPLETED = 4

# Global tour (cluster visit order)
uint32[] cluster_order

# Refined waypoints to follow
geometry_msgs/PoseStamped[] waypoints

# Current target
geometry_msgs/PoseStamped current_target

# Progress info
uint32 current_waypoint_index
uint32 total_waypoints
float64 estimated_time_remaining
float64 total_distance_remaining

# Target viewpoint info
float64 target_coverage         # Current target viewpoint coverage
uint32 target_cluster_id        # Current target cluster ID
uint32 target_cluster_size      # Current target cluster size