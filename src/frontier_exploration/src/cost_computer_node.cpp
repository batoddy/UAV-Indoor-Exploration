/**
 * Node 3: Cost Computer
 * 
 * Input:  /frontier_clusters_with_viewpoints (frontier_exploration/FrontierArray)
 *         /map (nav_msgs/OccupancyGrid)
 * Output: /frontier_clusters_complete (frontier_exploration/FrontierArray)
 * 
 * Computes connection costs between all cluster pairs based on:
 * - Path length (Euclidean or A* if needed)
 * - Yaw change
 * - Time lower bound = max(path/v_max, yaw_diff/yaw_rate_max)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "frontier_exploration/msg/frontier_array.hpp"
#include "frontier_exploration/msg/cluster_connection.hpp"
#include "frontier_exploration/common.hpp"

using namespace frontier_exploration;

class CostComputerNode : public rclcpp::Node
{
public:
  CostComputerNode() : Node("cost_computer")
  {
    // Parameters
    declare_parameter("v_max", 2.5);
    declare_parameter("yaw_rate_max", 3.0);
    
    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    
    // Subscribers
    clusters_sub_ = create_subscription<frontier_exploration::msg::FrontierArray>(
      "frontier_clusters_with_viewpoints", 10,
      std::bind(&CostComputerNode::clustersCallback, this, std::placeholders::_1));
    
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&CostComputerNode::mapCallback, this, std::placeholders::_1));
    
    // Publisher
    clusters_pub_ = create_publisher<frontier_exploration::msg::FrontierArray>(
      "frontier_clusters_complete", 10);
    
    RCLCPP_INFO(get_logger(), "Cost Computer initialized");
    RCLCPP_INFO(get_logger(), "  Input:  frontier_clusters_with_viewpoints, /map");
    RCLCPP_INFO(get_logger(), "  Output: frontier_clusters_complete");
    RCLCPP_INFO(get_logger(), "  v_max: %.2f m/s, yaw_rate_max: %.2f rad/s", v_max_, yaw_rate_max_);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    current_map_ = msg;
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
    output.clusters = msg->clusters;
    
    // Compute pairwise connection costs
    computeConnectionCosts(output.clusters);
    
    clusters_pub_->publish(output);
    
    auto duration = (now() - start_time).seconds() * 1000.0;
    RCLCPP_DEBUG(get_logger(), "Computed costs for %zu clusters in %.1f ms", 
                 output.clusters.size(), duration);
  }
  
  void computeConnectionCosts(std::vector<frontier_exploration::msg::FrontierCluster>& clusters)
  {
    // Clear existing connections
    for (auto& cluster : clusters) {
      cluster.connections.clear();
    }
    
    // Compute for all pairs
    for (size_t i = 0; i < clusters.size(); ++i) {
      for (size_t j = i + 1; j < clusters.size(); ++j) {
        auto& c1 = clusters[i];
        auto& c2 = clusters[j];
        
        // Skip if no viewpoints
        if (c1.viewpoints.empty() || c2.viewpoints.empty()) continue;
        
        // Use best viewpoints
        const auto& vp1 = c1.viewpoints[0];
        const auto& vp2 = c2.viewpoints[0];
        
        // Compute path length (Euclidean for now)
        double dx = vp2.position.x - vp1.position.x;
        double dy = vp2.position.y - vp1.position.y;
        double path_length = std::sqrt(dx * dx + dy * dy);
        
        // Yaw change
        double yaw_change = angleDiff(vp1.yaw, vp2.yaw);
        
        // Time lower bound
        double time_translation = path_length / v_max_;
        double time_rotation = yaw_change / yaw_rate_max_;
        double cost = std::max(time_translation, time_rotation);
        
        // Add bidirectional connections
        frontier_exploration::msg::ClusterConnection conn1;
        conn1.target_cluster_id = c2.id;
        conn1.cost = cost;
        conn1.path_length = path_length;
        conn1.yaw_change = yaw_change;
        c1.connections.push_back(conn1);
        
        frontier_exploration::msg::ClusterConnection conn2;
        conn2.target_cluster_id = c1.id;
        conn2.cost = cost;
        conn2.path_length = path_length;
        conn2.yaw_change = yaw_change;
        c2.connections.push_back(conn2);
      }
    }
  }
  
  double v_max_;
  double yaw_rate_max_;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  
  rclcpp::Subscription<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostComputerNode>());
  rclcpp::shutdown();
  return 0;
}