# Frontier Information Structure (FIS)
std_msgs/Header header

uint32 id
uint32 size

# Centroid in world coordinates
geometry_msgs/Point centroid

# Frontier cell positions (world coordinates)
geometry_msgs/Point[] cells

# Bounding box (grid coordinates)
int32 bbox_min_x
int32 bbox_min_y
int32 bbox_max_x
int32 bbox_max_y

# PCA results
float64[2] principal_axis
float64 principal_eigenvalue

# Viewpoints (filled by viewpoint_generator)
frontier_exploration/Viewpoint[] viewpoints

# Connection costs (filled by cost_computer)
frontier_exploration/ClusterConnection[] connections

# Visualization color
std_msgs/ColorRGBA color