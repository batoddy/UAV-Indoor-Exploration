# Table of all frontier clusters with costs
std_msgs/Header header
FrontierCluster[] clusters          # All frontier clusters
uint32 total_frontiers              # Total number of clusters
uint32 reachable_count              # Number of reachable clusters
uint32 selected_id                  # Currently selected goal cluster ID (0 if none)
