/**
 * Node 1: Frontier Detector
 * 
 * Input:  /map (nav_msgs/OccupancyGrid)
 * Output: /frontier_clusters (frontier_exploration/FrontierArray)
 * 
 * Detects frontier cells, clusters them with BFS, and splits large clusters with PCA.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "frontier_exploration/msg/frontier_array.hpp"
#include "frontier_exploration/msg/frontier_cluster.hpp"
#include "frontier_exploration/common.hpp"

#include <queue>
#include <algorithm>

using namespace frontier_exploration;

class FrontierDetectorNode : public rclcpp::Node
{
public:
  FrontierDetectorNode() : Node("frontier_detector")
  {
    // Parameters
    declare_parameter("map_topic", "/map");
    declare_parameter("min_frontier_size", 5);
    declare_parameter("max_cluster_size", 50);
    declare_parameter("free_threshold", 25);
    declare_parameter("occupied_threshold", 65);
    declare_parameter("pca_split_threshold", 2.0);
    
    map_topic_ = get_parameter("map_topic").as_string();
    min_frontier_size_ = get_parameter("min_frontier_size").as_int();
    max_cluster_size_ = get_parameter("max_cluster_size").as_int();
    free_threshold_ = get_parameter("free_threshold").as_int();
    occupied_threshold_ = get_parameter("occupied_threshold").as_int();
    pca_split_threshold_ = get_parameter("pca_split_threshold").as_double();
    
    // Subscriber
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, 10,
      std::bind(&FrontierDetectorNode::mapCallback, this, std::placeholders::_1));
    
    // Publisher
    clusters_pub_ = create_publisher<frontier_exploration::msg::FrontierArray>(
      "frontier_clusters", 10);
    
    color_palette_ = getColorPalette();
    
    RCLCPP_INFO(get_logger(), "Frontier Detector initialized");
    RCLCPP_INFO(get_logger(), "  Input:  %s", map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output: frontier_clusters");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    if (!map || map->data.empty()) return;
    
    auto start_time = now();
    
    // Reset cluster ID counter each detection cycle
    next_cluster_id_ = 0;
    
    // Detect frontiers
    auto clusters = detectFrontiers(map);
    
    // Publish
    frontier_exploration::msg::FrontierArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = map->header.frame_id;
    msg.clusters = clusters;
    
    clusters_pub_->publish(msg);
    
    auto duration = (now() - start_time).seconds() * 1000.0;
    RCLCPP_DEBUG(get_logger(), "Detected %zu clusters in %.1f ms", clusters.size(), duration);
  }
  
  std::vector<frontier_exploration::msg::FrontierCluster> detectFrontiers(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
  {
    std::vector<frontier_exploration::msg::FrontierCluster> clusters;
    
    int width = map->info.width;
    int height = map->info.height;
    std::vector<bool> visited(width * height, false);
    
    color_index_ = 0;
    
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        int idx = getIndex(x, y, width);
        
        if (!visited[idx] && isFrontierCell(x, y, *map)) {
          auto raw_cluster = buildCluster(x, y, *map, visited);
          
          if (static_cast<int>(raw_cluster.size()) >= min_frontier_size_) {
            auto split = recursiveSplit(raw_cluster, *map);
            
            for (auto& cells : split) {
              auto cluster_msg = createClusterMsg(cells, *map);
              clusters.push_back(cluster_msg);
            }
          }
        }
      }
    }
    
    return clusters;
  }
  
  bool isFrontierCell(int x, int y, const nav_msgs::msg::OccupancyGrid& map)
  {
    int width = map.info.width;
    int idx = getIndex(x, y, width);
    
    if (!isFree(map.data[idx], free_threshold_)) return false;
    
    for (const auto& [dx, dy] : NEIGHBORS_8) {
      int nidx = getIndex(x + dx, y + dy, width);
      if (isUnknown(map.data[nidx])) return true;
    }
    return false;
  }
  
  std::vector<std::pair<int,int>> buildCluster(int start_x, int start_y,
                                                const nav_msgs::msg::OccupancyGrid& map,
                                                std::vector<bool>& visited)
  {
    std::vector<std::pair<int,int>> cells;
    int width = map.info.width;
    int height = map.info.height;
    
    std::queue<std::pair<int,int>> queue;
    queue.push({start_x, start_y});
    visited[getIndex(start_x, start_y, width)] = true;
    
    while (!queue.empty()) {
      auto [x, y] = queue.front();
      queue.pop();
      
      if (isFrontierCell(x, y, map)) {
        cells.push_back({x, y});
        
        for (const auto& [dx, dy] : NEIGHBORS_8) {
          int nx = x + dx, ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          
          int nidx = getIndex(nx, ny, width);
          if (!visited[nidx]) {
            visited[nidx] = true;
            queue.push({nx, ny});
          }
        }
      }
    }
    return cells;
  }
  
  // PCA computation
  struct PCAResult {
    double mean_x, mean_y;
    double axis_x, axis_y;
    double eigenvalue;
  };
  
  PCAResult computePCA(const std::vector<std::pair<int,int>>& cells)
  {
    PCAResult result{0, 0, 1, 0, 0};
    if (cells.size() < 2) return result;
    
    // Mean
    for (const auto& [x, y] : cells) {
      result.mean_x += x;
      result.mean_y += y;
    }
    result.mean_x /= cells.size();
    result.mean_y /= cells.size();
    
    // Covariance
    double cov_xx = 0, cov_yy = 0, cov_xy = 0;
    for (const auto& [x, y] : cells) {
      double dx = x - result.mean_x;
      double dy = y - result.mean_y;
      cov_xx += dx * dx;
      cov_yy += dy * dy;
      cov_xy += dx * dy;
    }
    cov_xx /= cells.size();
    cov_yy /= cells.size();
    cov_xy /= cells.size();
    
    // Eigenvalue decomposition
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double discriminant = std::max(0.0, trace * trace - 4 * det);
    
    result.eigenvalue = (trace + std::sqrt(discriminant)) / 2.0;
    
    // Eigenvector
    if (std::abs(cov_xy) > 1e-6) {
      double vx = cov_xy;
      double vy = result.eigenvalue - cov_xx;
      double norm = std::sqrt(vx * vx + vy * vy);
      result.axis_x = vx / norm;
      result.axis_y = vy / norm;
    } else {
      result.axis_x = (cov_xx >= cov_yy) ? 1.0 : 0.0;
      result.axis_y = (cov_xx >= cov_yy) ? 0.0 : 1.0;
    }
    
    return result;
  }
  
  std::vector<std::vector<std::pair<int,int>>> splitCluster(
    const std::vector<std::pair<int,int>>& cells, const PCAResult& pca)
  {
    std::vector<std::pair<int,int>> cluster1, cluster2;
    
    for (const auto& [x, y] : cells) {
      double dx = x - pca.mean_x;
      double dy = y - pca.mean_y;
      double proj = dx * pca.axis_x + dy * pca.axis_y;
      
      if (proj <= 0) cluster1.push_back({x, y});
      else cluster2.push_back({x, y});
    }
    
    std::vector<std::vector<std::pair<int,int>>> result;
    if (!cluster1.empty()) result.push_back(cluster1);
    if (!cluster2.empty()) result.push_back(cluster2);
    return result;
  }
  
  std::vector<std::vector<std::pair<int,int>>> recursiveSplit(
    const std::vector<std::pair<int,int>>& cells,
    const nav_msgs::msg::OccupancyGrid& map)
  {
    std::vector<std::vector<std::pair<int,int>>> result;
    
    auto pca = computePCA(cells);
    
    if (static_cast<int>(cells.size()) <= max_cluster_size_ ||
        pca.eigenvalue < pca_split_threshold_) {
      result.push_back(cells);
      return result;
    }
    
    auto split = splitCluster(cells, pca);
    
    for (const auto& sub : split) {
      if (static_cast<int>(sub.size()) >= min_frontier_size_) {
        auto sub_result = recursiveSplit(sub, map);
        result.insert(result.end(), sub_result.begin(), sub_result.end());
      }
    }
    
    if (result.empty()) result.push_back(cells);
    return result;
  }
  
  frontier_exploration::msg::FrontierCluster createClusterMsg(
    const std::vector<std::pair<int,int>>& cells,
    const nav_msgs::msg::OccupancyGrid& map)
  {
    frontier_exploration::msg::FrontierCluster msg;
    msg.id = next_cluster_id_++;
    msg.size = cells.size();
    
    // Compute bbox and centroid
    int min_x = INT_MAX, min_y = INT_MAX;
    int max_x = INT_MIN, max_y = INT_MIN;
    double sum_x = 0, sum_y = 0;
    
    // Store cells in world coordinates
    msg.cells.reserve(cells.size());
    
    for (const auto& [x, y] : cells) {
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
      sum_x += x;
      sum_y += y;
      
      // Add cell position in world coordinates
      msg.cells.push_back(gridToWorld(x, y, map));
    }
    
    msg.bbox_min_x = min_x;
    msg.bbox_min_y = min_y;
    msg.bbox_max_x = max_x;
    msg.bbox_max_y = max_y;
    
    int cx = static_cast<int>(sum_x / cells.size());
    int cy = static_cast<int>(sum_y / cells.size());
    msg.centroid = gridToWorld(cx, cy, map);
    
    // PCA
    auto pca = computePCA(cells);
    msg.principal_axis[0] = pca.axis_x;
    msg.principal_axis[1] = pca.axis_y;
    msg.principal_eigenvalue = pca.eigenvalue;
    
    // Color
    msg.color = color_palette_[color_index_ % color_palette_.size()];
    color_index_++;
    
    return msg;
  }
  
  // Parameters
  std::string map_topic_;
  int min_frontier_size_;
  int max_cluster_size_;
  int8_t free_threshold_;
  int8_t occupied_threshold_;
  double pca_split_threshold_;
  
  uint32_t next_cluster_id_ = 0;
  size_t color_index_ = 0;
  std::vector<std_msgs::msg::ColorRGBA> color_palette_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierDetectorNode>());
  rclcpp::shutdown();
  return 0;
}