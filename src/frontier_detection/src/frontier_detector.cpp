#include "frontier_detection/frontier_detector.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace frontier_detection
{

namespace {
  std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a = 0.8f) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }
}

FrontierDetector::FrontierDetector()
{
  // Initialize color palette (distinct colors for clusters)
  color_palette_ = {
    createColor(1.0f, 0.0f, 0.0f),    // Red
    createColor(0.0f, 1.0f, 0.0f),    // Green
    createColor(0.0f, 0.0f, 1.0f),    // Blue
    createColor(1.0f, 1.0f, 0.0f),    // Yellow
    createColor(1.0f, 0.0f, 1.0f),    // Magenta
    createColor(0.0f, 1.0f, 1.0f),    // Cyan
    createColor(1.0f, 0.5f, 0.0f),    // Orange
    createColor(0.5f, 0.0f, 1.0f),    // Purple
    createColor(0.0f, 1.0f, 0.5f),    // Spring Green
    createColor(1.0f, 0.0f, 0.5f),    // Rose
    createColor(0.5f, 1.0f, 0.0f),    // Lime
    createColor(0.0f, 0.5f, 1.0f),    // Sky Blue
  };
}

std::vector<FrontierCluster> FrontierDetector::detectFrontiers(
  const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
  clusters_.clear();
  color_index_ = 0;
  
  if (!map || map->data.empty()) {
    return clusters_;
  }
  
  int width = map->info.width;
  int height = map->info.height;
  
  std::vector<bool> visited(width * height, false);
  
  // Scan entire map for frontier cells
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int idx = getIndex(x, y, width);
      
      if (!visited[idx] && isFrontierCell(x, y, *map)) {
        // Found a new frontier - build cluster with BFS
        FrontierCluster cluster = buildCluster(x, y, *map, visited);
        
        if (static_cast<int>(cluster.cells.size()) >= min_frontier_size_) {
          // Compute properties
          computeClusterProperties(cluster, *map);
          
          // PCA-based recursive splitting if cluster is too large
          auto split_clusters = recursiveSplit(cluster, *map);
          
          for (auto& sc : split_clusters) {
            sc.id = next_cluster_id_++;
            assignColor(sc);
            clusters_.push_back(sc);
          }
        }
      }
    }
  }
  
  return clusters_;
}

std::vector<FrontierCluster> FrontierDetector::updateFrontiers(
  const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
  const AABB& updated_region)
{
  if (!map || map->data.empty()) {
    return clusters_;
  }
  
  // Step 1: Remove clusters affected by the update (broad phase: AABB check)
  std::vector<FrontierCluster> unaffected;
  for (const auto& cluster : clusters_) {
    if (!cluster.bbox.intersects(updated_region)) {
      unaffected.push_back(cluster);
    } else {
      // Narrow phase: check if any cell is no longer a frontier
      bool still_valid = true;
      for (const auto& [cx, cy] : cluster.cells) {
        if (!isFrontierCell(cx, cy, *map)) {
          still_valid = false;
          break;
        }
      }
      if (still_valid) {
        unaffected.push_back(cluster);
      }
    }
  }
  
  clusters_ = unaffected;
  
  // Step 2: Search for new frontiers only in updated region
  int width = map->info.width;
  int height = map->info.height;
  
  std::vector<bool> visited(width * height, false);
  
  // Mark existing cluster cells as visited
  for (const auto& cluster : clusters_) {
    for (const auto& [cx, cy] : cluster.cells) {
      visited[getIndex(cx, cy, width)] = true;
    }
  }
  
  // Scan updated region
  int scan_min_x = std::max(1, updated_region.min_x);
  int scan_max_x = std::min(width - 2, updated_region.max_x);
  int scan_min_y = std::max(1, updated_region.min_y);
  int scan_max_y = std::min(height - 2, updated_region.max_y);
  
  for (int y = scan_min_y; y <= scan_max_y; ++y) {
    for (int x = scan_min_x; x <= scan_max_x; ++x) {
      int idx = getIndex(x, y, width);
      
      if (!visited[idx] && isFrontierCell(x, y, *map)) {
        FrontierCluster cluster = buildCluster(x, y, *map, visited);
        
        if (static_cast<int>(cluster.cells.size()) >= min_frontier_size_) {
          computeClusterProperties(cluster, *map);
          
          auto split_clusters = recursiveSplit(cluster, *map);
          
          for (auto& sc : split_clusters) {
            sc.id = next_cluster_id_++;
            assignColor(sc);
            clusters_.push_back(sc);
          }
        }
      }
    }
  }
  
  return clusters_;
}

FrontierCluster FrontierDetector::buildCluster(int start_x, int start_y,
                                                const nav_msgs::msg::OccupancyGrid& map,
                                                std::vector<bool>& visited)
{
  FrontierCluster cluster;
  cluster.size = 0;
  cluster.bbox.reset();
  
  int width = map.info.width;
  int height = map.info.height;
  
  std::queue<std::pair<int, int>> queue;
  queue.push({start_x, start_y});
  visited[getIndex(start_x, start_y, width)] = true;
  
  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();
    
    if (isFrontierCell(x, y, map)) {
      cluster.cells.push_back({x, y});
      cluster.bbox.expand(x, y);
      
      // Check neighbors for more frontier cells
      for (const auto& [dx, dy] : neighbors_) {
        int nx = x + dx;
        int ny = y + dy;
        
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        
        int nidx = getIndex(nx, ny, width);
        if (!visited[nidx]) {
          visited[nidx] = true;
          queue.push({nx, ny});
        }
      }
    }
  }
  
  cluster.size = cluster.cells.size();
  return cluster;
}

void FrontierDetector::computeClusterProperties(FrontierCluster& cluster,
                                                 const nav_msgs::msg::OccupancyGrid& map)
{
  if (cluster.cells.empty()) return;
  
  // Compute centroid
  double sum_x = 0, sum_y = 0;
  for (const auto& [x, y] : cluster.cells) {
    sum_x += x;
    sum_y += y;
  }
  
  double cx = sum_x / cluster.cells.size();
  double cy = sum_y / cluster.cells.size();
  
  cluster.centroid = gridToWorld(static_cast<int>(cx), static_cast<int>(cy), map);
  
  // Compute PCA
  computePCA(cluster);
}

void FrontierDetector::computePCA(FrontierCluster& cluster)
{
  if (cluster.cells.size() < 2) {
    cluster.principal_axis = {1.0, 0.0};
    cluster.principal_eigenvalue = 0.0;
    return;
  }
  
  // Compute mean
  double mean_x = 0, mean_y = 0;
  for (const auto& [x, y] : cluster.cells) {
    mean_x += x;
    mean_y += y;
  }
  mean_x /= cluster.cells.size();
  mean_y /= cluster.cells.size();
  
  // Compute covariance matrix
  // [cov_xx  cov_xy]
  // [cov_xy  cov_yy]
  double cov_xx = 0, cov_yy = 0, cov_xy = 0;
  for (const auto& [x, y] : cluster.cells) {
    double dx = x - mean_x;
    double dy = y - mean_y;
    cov_xx += dx * dx;
    cov_yy += dy * dy;
    cov_xy += dx * dy;
  }
  cov_xx /= cluster.cells.size();
  cov_yy /= cluster.cells.size();
  cov_xy /= cluster.cells.size();
  
  // Eigenvalue decomposition for 2x2 symmetric matrix
  // λ = (trace ± sqrt(trace² - 4*det)) / 2
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double discriminant = trace * trace - 4 * det;
  
  if (discriminant < 0) discriminant = 0;
  
  double lambda1 = (trace + std::sqrt(discriminant)) / 2.0;  // Larger eigenvalue
  // double lambda2 = (trace - std::sqrt(discriminant)) / 2.0;  // Smaller eigenvalue
  
  cluster.principal_eigenvalue = lambda1;
  
  // Compute eigenvector for lambda1
  // (A - λI)v = 0
  // (cov_xx - λ)vx + cov_xy * vy = 0
  if (std::abs(cov_xy) > 1e-6) {
    double vx = cov_xy;
    double vy = lambda1 - cov_xx;
    double norm = std::sqrt(vx * vx + vy * vy);
    cluster.principal_axis = {vx / norm, vy / norm};
  } else {
    // Diagonal matrix - principal axis is along larger variance
    if (cov_xx >= cov_yy) {
      cluster.principal_axis = {1.0, 0.0};
    } else {
      cluster.principal_axis = {0.0, 1.0};
    }
  }
}

std::vector<FrontierCluster> FrontierDetector::recursiveSplit(
  const FrontierCluster& cluster,
  const nav_msgs::msg::OccupancyGrid& map)
{
  std::vector<FrontierCluster> result;
  
  // Base case: cluster is small enough or can't be split meaningfully
  if (static_cast<int>(cluster.cells.size()) <= max_cluster_size_ ||
      cluster.principal_eigenvalue < pca_split_threshold_) {
    result.push_back(cluster);
    return result;
  }
  
  // Split along principal axis
  auto split = splitClusterPCA(cluster, map);
  
  // Recursively split each part
  for (const auto& sub_cluster : split) {
    if (static_cast<int>(sub_cluster.cells.size()) >= min_frontier_size_) {
      auto sub_result = recursiveSplit(sub_cluster, map);
      result.insert(result.end(), sub_result.begin(), sub_result.end());
    }
  }
  
  // If splitting failed, return original
  if (result.empty()) {
    result.push_back(cluster);
  }
  
  return result;
}

std::vector<FrontierCluster> FrontierDetector::splitClusterPCA(
  const FrontierCluster& cluster,
  const nav_msgs::msg::OccupancyGrid& map)
{
  std::vector<FrontierCluster> result;
  
  if (cluster.cells.size() < 2) {
    result.push_back(cluster);
    return result;
  }
  
  // Compute mean position
  double mean_x = 0, mean_y = 0;
  for (const auto& [x, y] : cluster.cells) {
    mean_x += x;
    mean_y += y;
  }
  mean_x /= cluster.cells.size();
  mean_y /= cluster.cells.size();
  
  // Split cells based on projection onto principal axis
  // Project each point: proj = (p - mean) · principal_axis
  FrontierCluster cluster1, cluster2;
  cluster1.bbox.reset();
  cluster2.bbox.reset();
  
  for (const auto& [x, y] : cluster.cells) {
    double dx = x - mean_x;
    double dy = y - mean_y;
    double proj = dx * cluster.principal_axis[0] + dy * cluster.principal_axis[1];
    
    if (proj <= 0) {
      cluster1.cells.push_back({x, y});
      cluster1.bbox.expand(x, y);
    } else {
      cluster2.cells.push_back({x, y});
      cluster2.bbox.expand(x, y);
    }
  }
  
  // Compute properties for split clusters
  if (!cluster1.cells.empty()) {
    cluster1.size = cluster1.cells.size();
    computeClusterProperties(cluster1, map);
    result.push_back(cluster1);
  }
  
  if (!cluster2.cells.empty()) {
    cluster2.size = cluster2.cells.size();
    computeClusterProperties(cluster2, map);
    result.push_back(cluster2);
  }
  
  return result;
}

void FrontierDetector::assignColor(FrontierCluster& cluster)
{
  cluster.color = color_palette_[color_index_ % color_palette_.size()];
  color_index_++;
}

bool FrontierDetector::isFrontierCell(int x, int y, const nav_msgs::msg::OccupancyGrid& map)
{
  int width = map.info.width;
  int idx = getIndex(x, y, width);
  
  // Frontier cell must be FREE
  if (!isFree(map.data[idx])) {
    return false;
  }
  
  // And have at least one UNKNOWN neighbor
  for (const auto& [dx, dy] : neighbors_) {
    int nx = x + dx;
    int ny = y + dy;
    int nidx = getIndex(nx, ny, width);
    
    if (isUnknown(map.data[nidx])) {
      return true;
    }
  }
  
  return false;
}

bool FrontierDetector::isFree(int8_t value) const
{
  return value >= 0 && value < free_threshold_;
}

bool FrontierDetector::isUnknown(int8_t value) const
{
  return value == -1;
}

int FrontierDetector::getIndex(int x, int y, int width) const
{
  return y * width + x;
}

geometry_msgs::msg::Point FrontierDetector::gridToWorld(int x, int y, 
                                                         const nav_msgs::msg::OccupancyGrid& map)
{
  geometry_msgs::msg::Point point;
  point.x = map.info.origin.position.x + (x + 0.5) * map.info.resolution;
  point.y = map.info.origin.position.y + (y + 0.5) * map.info.resolution;
  point.z = 0.0;
  return point;
}

}  // namespace frontier_detection