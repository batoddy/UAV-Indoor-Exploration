# A cluster of frontier points with computed centroid
uint32 id                           # Unique cluster ID
geometry_msgs/Point centroid        # Cluster center (goal candidate)
geometry_msgs/Point[] points        # All points in this cluster
uint32 size                         # Number of points
float64 cost                        # Computed cost (lower = better)
float64 utility                     # Exploration utility score
bool visited                        # Has this frontier been visited?
bool reachable                      # Can we plan a path to it?
