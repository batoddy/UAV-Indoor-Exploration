#ifndef FRONTIER_DETECTION__FRONTIER_DETECTOR_HPP_
#define FRONTIER_DETECTION__FRONTIER_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <vector>
#include <utility>
#include <array>
#include <cstdint>

namespace frontier_detection
{

// Axis-Aligned Bounding Box
struct AABB
{
  int min_x, min_y;
  int max_x, max_y;
  
  bool intersects(const AABB& other) const {
    return !(max_x < other.min_x || min_x > other.max_x ||
             max_y < other.min_y || min_y > other.max_y);
  }
  
  void expand(int x, int y) {
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  }
  
  void reset() {
    min_x = min_y = std::numeric_limits<int>::max();
    max_x = max_y = std::numeric_limits<int>::min();
  }
};

// Frontier Information Structure (FIS) - inspired by FUEL paper
struct FrontierCluster
{
  uint32_t id;                              // Unique cluster ID
  std::vector<std::pair<int, int>> cells;   // Grid coordinates
  geometry_msgs::msg::Point centroid;       // World coordinates (pavg)
  AABB bbox;                                // Axis-aligned bounding box
  double size;                              // Number of cells
  
  // PCA results
  std::array<double, 2> principal_axis;     // First principal component direction
  double principal_eigenvalue;              // Largest eigenvalue (spread along principal axis)
  
  // Visualization
  std_msgs::msg::ColorRGBA color;
};

class FrontierDetector
{
public:
  FrontierDetector();
  
  // Main detection function
  std::vector<FrontierCluster> detectFrontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
  
  // Incremental update (only process changed region)
  std::vector<FrontierCluster> updateFrontiers(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const AABB& updated_region);
  
  // Parameters
  void setMinFrontierSize(int size) { min_frontier_size_ = size; }
  void setMaxClusterSize(int size) { max_cluster_size_ = size; }
  void setFreeThreshold(int8_t thresh) { free_threshold_ = thresh; }
  void setOccupiedThreshold(int8_t thresh) { occupied_threshold_ = thresh; }
  void setPcaSplitThreshold(double thresh) { pca_split_threshold_ = thresh; }

  // Getters
  const std::vector<FrontierCluster>& getClusters() const { return clusters_; }

private:
  // Check if cell is a frontier cell
  bool isFrontierCell(int x, int y, const nav_msgs::msg::OccupancyGrid& map);
  
  // Check cell type
  bool isFree(int8_t value) const;
  bool isUnknown(int8_t value) const;
  
  // Get cell index from coordinates
  int getIndex(int x, int y, int width) const;
  
  // Convert grid to world coordinates
  geometry_msgs::msg::Point gridToWorld(int x, int y, const nav_msgs::msg::OccupancyGrid& map);
  
  // BFS to find connected frontier cells
  FrontierCluster buildCluster(int start_x, int start_y, 
                               const nav_msgs::msg::OccupancyGrid& map,
                               std::vector<bool>& visited);
  
  // PCA analysis for a cluster
  void computePCA(FrontierCluster& cluster);
  
  // Split cluster along principal axis if too large
  std::vector<FrontierCluster> splitClusterPCA(const FrontierCluster& cluster,
                                                const nav_msgs::msg::OccupancyGrid& map);
  
  // Recursive splitting until all clusters are small enough
  std::vector<FrontierCluster> recursiveSplit(const FrontierCluster& cluster,
                                               const nav_msgs::msg::OccupancyGrid& map);
  
  // Assign unique color to cluster
  void assignColor(FrontierCluster& cluster);
  
  // Compute cluster properties (centroid, bbox, etc.)
  void computeClusterProperties(FrontierCluster& cluster,
                                 const nav_msgs::msg::OccupancyGrid& map);
  
  // Stored clusters for incremental updates
  std::vector<FrontierCluster> clusters_;
  uint32_t next_cluster_id_ = 0;
  
  // Parameters
  int min_frontier_size_ = 5;
  int max_cluster_size_ = 50;        // Max cells before PCA split
  int8_t free_threshold_ = 25;
  int8_t occupied_threshold_ = 65;
  double pca_split_threshold_ = 2.0; // Eigenvalue ratio threshold for splitting
  
  // Color palette for visualization
  std::vector<std_msgs::msg::ColorRGBA> color_palette_;
  size_t color_index_ = 0;
  
  // 8-connected neighbors
  const std::vector<std::pair<int, int>> neighbors_ = {
    {-1, -1}, {0, -1}, {1, -1},
    {-1,  0},          {1,  0},
    {-1,  1}, {0,  1}, {1,  1}
  };
};

}  // namespace frontier_detection

#endif  // FRONTIER_DETECTION__FRONTIER_DETECTOR_HPP_
