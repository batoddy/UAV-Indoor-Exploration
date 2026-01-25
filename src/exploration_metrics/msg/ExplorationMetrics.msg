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
float64 estimated_completion_time  # Estimated time to 100% (-1 if not calculable)

# ============================================================================
# OCCUPANCY GRID METRICS (2D)
# ============================================================================
int32 gt_known_cells               # Known cells in ground truth map
int32 current_known_cells          # Currently known cells
int32 matched_cells                # Cells matching ground truth
float64 occupancy_grid_percentage  # 2D exploration percentage

# ============================================================================
# OCTOMAP METRICS (3D)
# ============================================================================
int32 gt_known_voxels              # Known voxels in ground truth OctoMap
int32 current_known_voxels         # Currently known voxels
int32 matched_voxels               # Voxels matching ground truth
float64 octomap_percentage         # 3D exploration percentage

# ============================================================================
# SPACE RATIOS
# ============================================================================
float64 free_space_ratio           # Ratio of free space (0-1)
float64 occupied_space_ratio       # Ratio of occupied space (0-1)
float64 unknown_space_ratio        # Ratio of unknown space (0-1)

# ============================================================================
# IOU METRICS
# ============================================================================
float64 iou_free                   # IoU for free space
float64 iou_occupied               # IoU for occupied space

# Comparison mode indicator
string comparison_mode             # "occupancy_grid", "octomap", or "both"
