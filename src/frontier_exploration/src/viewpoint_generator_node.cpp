/**
 * Node 2: Viewpoint Generator
 * 
 * Input:  /frontier_clusters (frontier_exploration/FrontierArray)
 *         /octomap_binary (octomap_msgs/Octomap) - 3D obstacle checking
 *         /map (nav_msgs/OccupancyGrid) - 2D fallback
 * Output: /frontier_clusters_with_viewpoints (frontier_exploration/FrontierArray)
 * 
 * Generates candidate viewpoints for each frontier cluster using cylindrical sampling.
 * Uses OctoMap for accurate 3D collision checking with robot footprint.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include "frontier_exploration/msg/frontier_array.hpp"
#include "frontier_exploration/msg/viewpoint.hpp"
#include "frontier_exploration/common.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

using namespace frontier_exploration;

class ViewpointGeneratorNode : public rclcpp::Node
{
public:
  ViewpointGeneratorNode() : Node("viewpoint_generator")
  {
    // Sensor parameters
    declare_parameter("sensor_range", 5.0);
    declare_parameter("sensor_fov_h", 1.57);
    
    // Viewpoint sampling parameters
    declare_parameter("min_dist", 1.5);
    declare_parameter("max_dist", 4.0);
    declare_parameter("num_dist_samples", 3);
    declare_parameter("num_angle_samples", 12);
    declare_parameter("min_coverage", 3);
    declare_parameter("max_viewpoints", 5);
    
    // Map thresholds
    declare_parameter("free_threshold", 25);
    declare_parameter("occupied_threshold", 65);
    
    // Robot footprint parameters
    declare_parameter("robot_width", 0.5);       // [m] Robot width (X)
    declare_parameter("robot_length", 0.5);      // [m] Robot length (Y)  
    declare_parameter("robot_height", 0.3);      // [m] Robot height (Z) - for 3D check
    declare_parameter("safety_margin", 0.3);     // [m] Extra clearance around robot
    
    // Flight height for 3D checking
    declare_parameter("flight_height", 1.0);     // [m] Nominal flight height
    declare_parameter("height_tolerance", 0.5);  // [m] Check this range around flight height
    
    sensor_range_ = get_parameter("sensor_range").as_double();
    sensor_fov_h_ = get_parameter("sensor_fov_h").as_double();
    min_dist_ = get_parameter("min_dist").as_double();
    max_dist_ = get_parameter("max_dist").as_double();
    num_dist_samples_ = get_parameter("num_dist_samples").as_int();
    num_angle_samples_ = get_parameter("num_angle_samples").as_int();
    min_coverage_ = get_parameter("min_coverage").as_int();
    max_viewpoints_ = get_parameter("max_viewpoints").as_int();
    free_threshold_ = get_parameter("free_threshold").as_int();
    occupied_threshold_ = get_parameter("occupied_threshold").as_int();
    
    robot_width_ = get_parameter("robot_width").as_double();
    robot_length_ = get_parameter("robot_length").as_double();
    robot_height_ = get_parameter("robot_height").as_double();
    safety_margin_ = get_parameter("safety_margin").as_double();
    flight_height_ = get_parameter("flight_height").as_double();
    height_tolerance_ = get_parameter("height_tolerance").as_double();
    
    // Calculate clearance radius (diagonal/2 + safety margin)
    clearance_radius_ = std::sqrt(robot_width_*robot_width_ + robot_length_*robot_length_) / 2.0 
                        + safety_margin_;
    
    // Subscribers
    clusters_sub_ = create_subscription<frontier_exploration::msg::FrontierArray>(
      "frontier_clusters", 10,
      std::bind(&ViewpointGeneratorNode::clustersCallback, this, std::placeholders::_1));
    
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&ViewpointGeneratorNode::mapCallback, this, std::placeholders::_1));
    
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", 10,
      std::bind(&ViewpointGeneratorNode::octomapCallback, this, std::placeholders::_1));
    
    // Publisher
    clusters_pub_ = create_publisher<frontier_exploration::msg::FrontierArray>(
      "frontier_clusters_with_viewpoints", 10);
    
    RCLCPP_INFO(get_logger(), "Viewpoint Generator initialized");
    RCLCPP_INFO(get_logger(), "  Robot: %.2f x %.2f x %.2f m", robot_width_, robot_length_, robot_height_);
    RCLCPP_INFO(get_logger(), "  Clearance radius: %.2f m (includes %.2f m safety)", 
                clearance_radius_, safety_margin_);
    RCLCPP_INFO(get_logger(), "  Flight height: %.2f m (±%.2f m)", flight_height_, height_tolerance_);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    current_map_ = msg;
  }
  
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    // Convert to OcTree
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    if (abstract_tree) {
      current_octree_.reset(dynamic_cast<octomap::OcTree*>(abstract_tree));
      if (!current_octree_) {
        delete abstract_tree;
      }
    }
  }
  
  void clustersCallback(const frontier_exploration::msg::FrontierArray::SharedPtr msg)
  {
    if (!current_map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No map received yet");
      return;
    }
    
    auto start_time = now();
    
    frontier_exploration::msg::FrontierArray output;
    output.header = msg->header;
    
    int total_candidates = 0;
    int rejected_2d = 0;
    int rejected_3d = 0;
    int accepted = 0;
    
    for (auto cluster : msg->clusters) {
      auto [t_cand, r_2d, r_3d, acc] = generateViewpoints(cluster, *current_map_);
      total_candidates += t_cand;
      rejected_2d += r_2d;
      rejected_3d += r_3d;
      accepted += acc;
      output.clusters.push_back(cluster);
    }
    
    clusters_pub_->publish(output);
    
    auto duration = (now() - start_time).seconds() * 1000.0;
    RCLCPP_DEBUG(get_logger(), 
                 "Viewpoints: %d candidates, %d rejected(2D), %d rejected(3D), %d accepted [%.1fms]",
                 total_candidates, rejected_2d, rejected_3d, accepted, duration);
  }
  
  std::tuple<int, int, int, int> generateViewpoints(
    frontier_exploration::msg::FrontierCluster& cluster,
    const nav_msgs::msg::OccupancyGrid& map)
  {
    cluster.viewpoints.clear();
    
    double dist_step = (num_dist_samples_ > 1) ? 
      (max_dist_ - min_dist_) / (num_dist_samples_ - 1) : 0;
    double angle_step = 2.0 * M_PI / num_angle_samples_;
    
    std::vector<frontier_exploration::msg::Viewpoint> candidates;
    
    int total_candidates = 0;
    int rejected_2d = 0;
    int rejected_3d = 0;
    
    for (int di = 0; di < num_dist_samples_; ++di) {
      double dist = min_dist_ + di * dist_step;
      
      for (int ai = 0; ai < num_angle_samples_; ++ai) {
        double angle = ai * angle_step;
        total_candidates++;
        
        geometry_msgs::msg::Point vp_pos;
        vp_pos.x = cluster.centroid.x + dist * std::cos(angle);
        vp_pos.y = cluster.centroid.y + dist * std::sin(angle);
        vp_pos.z = flight_height_;
        
        // Step 1: Check 2D map clearance
        if (!hasFootprintClearance2D(vp_pos.x, vp_pos.y, map)) {
          rejected_2d++;
          continue;
        }
        
        // Step 2: Check 3D OctoMap clearance (if available)
        if (current_octree_ && !hasFootprintClearance3D(vp_pos.x, vp_pos.y, vp_pos.z)) {
          rejected_3d++;
          continue;
        }
        
        // Step 3: Optimize yaw for coverage
        auto [best_yaw, best_coverage] = optimizeYaw(vp_pos, cluster, map);
        
        if (best_coverage >= min_coverage_) {
          frontier_exploration::msg::Viewpoint vp;
          vp.position = vp_pos;
          vp.yaw = best_yaw;
          vp.coverage = best_coverage;
          vp.distance_to_centroid = dist;
          candidates.push_back(vp);
        }
      }
    }
    
    // Sort by coverage (descending)
    std::sort(candidates.begin(), candidates.end(),
      [](const auto& a, const auto& b) { return a.coverage > b.coverage; });
    
    // Keep top N
    int n = std::min(static_cast<int>(candidates.size()), max_viewpoints_);
    cluster.viewpoints.assign(candidates.begin(), candidates.begin() + n);
    
    return {total_candidates, rejected_2d, rejected_3d, static_cast<int>(cluster.viewpoints.size())};
  }
  
  /**
   * Check 2D clearance using OccupancyGrid
   */
  bool hasFootprintClearance2D(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map)
  {
    auto [center_gx, center_gy] = worldToGrid(wx, wy, map);
    
    int width = map.info.width;
    int height = map.info.height;
    double resolution = map.info.resolution;
    
    int cell_radius = static_cast<int>(std::ceil(clearance_radius_ / resolution));
    
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
        int gx = center_gx + dx;
        int gy = center_gy + dy;
        
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
          return false;
        }
        
        double cell_dist = std::sqrt(dx*dx + dy*dy) * resolution;
        if (cell_dist > clearance_radius_) {
          continue;
        }
        
        int8_t cell_value = map.data[getIndex(gx, gy, width)];
        
        if (isOccupied(cell_value, occupied_threshold_)) {
          return false;
        }
        
        // Reject unknown cells within half clearance
        if (cell_value < 0 && cell_dist < clearance_radius_ * 0.5) {
          return false;
        }
      }
    }
    
    int center_idx = getIndex(center_gx, center_gy, width);
    return isFree(map.data[center_idx], free_threshold_);
  }
  
  /**
   * Check 3D clearance using OctoMap
   * Creates a bounding box around robot and checks for occupied voxels
   */
  bool hasFootprintClearance3D(double wx, double wy, double wz)
  {
    if (!current_octree_) return true;  // No octomap, assume OK
    
    double resolution = current_octree_->getResolution();
    
    // Define bounding box with safety margin
    double half_w = (robot_width_ / 2.0) + safety_margin_;
    double half_l = (robot_length_ / 2.0) + safety_margin_;
    double half_h = (robot_height_ / 2.0) + height_tolerance_;
    
    // Sample points in the bounding box
    double step = resolution;  // Check at octomap resolution
    
    for (double dz = -half_h; dz <= half_h; dz += step) {
      for (double dy = -half_l; dy <= half_l; dy += step) {
        for (double dx = -half_w; dx <= half_w; dx += step) {
          double px = wx + dx;
          double py = wy + dy;
          double pz = wz + dz;
          
          octomap::OcTreeNode* node = current_octree_->search(px, py, pz);
          
          if (node && current_octree_->isNodeOccupied(node)) {
            return false;  // Collision detected
          }
        }
      }
    }
    
    return true;  // No collision
  }
  
  std::pair<double, int> optimizeYaw(const geometry_msgs::msg::Point& vp_pos,
                                      const frontier_exploration::msg::FrontierCluster& cluster,
                                      const nav_msgs::msg::OccupancyGrid& map)
  {
    const int num_samples = 36;
    double best_yaw = 0;
    int best_coverage = -1;
    
    for (int i = 0; i < num_samples; ++i) {
      double yaw = (2.0 * M_PI * i) / num_samples - M_PI;
      int coverage = computeCoverage(vp_pos, yaw, cluster, map);
      
      if (coverage > best_coverage) {
        best_coverage = coverage;
        best_yaw = yaw;
      }
    }
    
    return {best_yaw, best_coverage};
  }
  
  int computeCoverage(const geometry_msgs::msg::Point& vp_pos, double yaw,
                      const frontier_exploration::msg::FrontierCluster& cluster,
                      const nav_msgs::msg::OccupancyGrid& map)
  {
    int count = 0;
    
    for (int gy = cluster.bbox_min_y; gy <= cluster.bbox_max_y; ++gy) {
      for (int gx = cluster.bbox_min_x; gx <= cluster.bbox_max_x; ++gx) {
        auto cell = gridToWorld(gx, gy, map);
        
        double dx = cell.x - vp_pos.x;
        double dy = cell.y - vp_pos.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist > sensor_range_) continue;
        
        double angle_to_cell = std::atan2(dy, dx);
        if (angleDiff(angle_to_cell, yaw) > sensor_fov_h_ / 2.0) continue;
        
        if (hasLineOfSight(vp_pos.x, vp_pos.y, cell.x, cell.y, map)) {
          count++;
        }
      }
    }
    
    return count;
  }
  
  bool hasLineOfSight(double x1, double y1, double x2, double y2,
                      const nav_msgs::msg::OccupancyGrid& map)
  {
    auto [gx1, gy1] = worldToGrid(x1, y1, map);
    auto [gx2, gy2] = worldToGrid(x2, y2, map);
    
    int dx = std::abs(gx2 - gx1);
    int dy = std::abs(gy2 - gy1);
    int sx = (gx1 < gx2) ? 1 : -1;
    int sy = (gy1 < gy2) ? 1 : -1;
    int err = dx - dy;
    
    int x = gx1, y = gy1;
    int width = map.info.width;
    int height = map.info.height;
    
    while (true) {
      if (x >= 0 && x < width && y >= 0 && y < height) {
        if (isOccupied(map.data[getIndex(x, y, width)], occupied_threshold_)) {
          return false;
        }
      }
      
      if (x == gx2 && y == gy2) break;
      
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
    }
    
    return true;
  }
  
  // Parameters
  double sensor_range_, sensor_fov_h_;
  double min_dist_, max_dist_;
  int num_dist_samples_, num_angle_samples_;
  int min_coverage_, max_viewpoints_;
  int8_t free_threshold_, occupied_threshold_;
  
  // Robot footprint
  double robot_width_, robot_length_, robot_height_;
  double safety_margin_;
  double clearance_radius_;
  double flight_height_, height_tolerance_;
  
  // Maps
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::shared_ptr<octomap::OcTree> current_octree_;
  
  // ROS
  rclcpp::Subscription<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViewpointGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}