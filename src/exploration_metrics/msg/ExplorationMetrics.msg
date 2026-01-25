# Exploration Metrics Message
# All metrics logged to CSV are also published here

std_msgs/Header header

# Timing metrics
uint64 tick                        # Simple counter incrementing each publish
float64 elapsed_time               # Time since node started (seconds)
float64 exploration_duration       # Time since exploration started (seconds, -1 if not started)

# ============================================================================
# MOTION METRICS (from telemetry)
# ============================================================================
float64 velocity_instant           # [m/s] Instantaneous horizontal velocity
float64 velocity_average           # [m/s] Average velocity (EMA)
float64 acceleration               # [m/s²] Instantaneous acceleration magnitude
float64 yaw_rad                    # [rad] Current yaw angle
float64 yaw_deg                    # [deg] Current yaw angle in degrees
float64 path_traveled              # [m] Total path traveled since start

# ============================================================================
# EXPLORATION METRICS
# ============================================================================
float64 exploration_percentage     # Overall exploration percentage (0-100)
float64 exploration_rate           # Instantaneous exploration rate (%/second)
float64 avg_exploration_rate       # Average exploration rate (EMA)

# ============================================================================
# MAP EXPLORATION PERCENTAGES
# ============================================================================
float64 exploration_2d_percentage  # 2D map exploration percentage (0-100)
float64 exploration_3d_percentage  # 3D map exploration percentage (0-100)

# Comparison mode indicator
string comparison_mode             # "occupancy_grid", "octomap", or "both"
