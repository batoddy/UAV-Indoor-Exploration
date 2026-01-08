/**
 * Node 2: Local Viewpoint Refiner
 * 
 * Input:  /exploration/global_tour (exploration_planner/ExplorationStatus)
 *         /frontier_clusters_complete (frontier_exploration/FrontierArray)
 *         /odom (current position)
 * Output: /exploration/refined_tour (exploration_planner/ExplorationStatus)
 * 
 * Refines local segment of global tour by considering multiple viewpoints
 * per cluster using Dijkstra graph search.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "frontier_exploration/msg/frontier_array.hpp"
#include "exploration_planner/msg/exploration_status.hpp"
#include "exploration_planner/common.hpp"

#include <queue>
#include <unordered_map>

using namespace exploration_planner;

class LocalRefinerNode : public rclcpp::Node
{
public:
  LocalRefinerNode() : Node("local_refiner")
  {
    // Parameters
    declare_parameter("refine_radius", 5.0);      // R_rf in paper
    declare_parameter("max_refine_clusters", 3);  // N_rf in paper
    declare_parameter("v_max", 1.0);
    declare_parameter("yaw_rate_max", 1.0);
    declare_parameter("w_consistency", 0.5);
    
    refine_radius_ = get_parameter("refine_radius").as_double();
    max_refine_clusters_ = get_parameter("max_refine_clusters").as_int();
    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    w_consistency_ = get_parameter("w_consistency").as_double();
    
    // Subscribers
    global_tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10,
      std::bind(&LocalRefinerNode::tourCallback, this, std::placeholders::_1));
    
    clusters_sub_ = create_subscription<frontier_exploration::msg::FrontierArray>(
      "/frontier_clusters_complete", 10,
      std::bind(&LocalRefinerNode::clustersCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&LocalRefinerNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    refined_pub_ = create_publisher<exploration_planner::msg::ExplorationStatus>(
      "/exploration/refined_tour", 10);
    
    RCLCPP_INFO(get_logger(), "Local Refiner initialized");
    RCLCPP_INFO(get_logger(), "  refine_radius: %.1f m, max_clusters: %d", 
                refine_radius_, max_refine_clusters_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    current_velocity_ = msg->twist.twist;
    have_pose_ = true;
  }
  
  void clustersCallback(const frontier_exploration::msg::FrontierArray::SharedPtr msg)
  {
    current_clusters_ = msg;
  }
  
  void tourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    if (!have_pose_ || !current_clusters_) {
      // Just pass through if we can't refine
      refined_pub_->publish(*msg);
      return;
    }
    
    if (msg->waypoints.empty() || 
        msg->state == exploration_planner::msg::ExplorationStatus::COMPLETED) {
      refined_pub_->publish(*msg);
      return;
    }
    
    auto start_time = now();
    
    // Find clusters within refine_radius
    geometry_msgs::msg::Point current_pos = current_pose_.pose.position;
    
    std::vector<size_t> nearby_indices;
    for (size_t i = 0; i < msg->waypoints.size() && 
                        static_cast<int>(nearby_indices.size()) < max_refine_clusters_; ++i) {
      double dist = distance2D(current_pos, msg->waypoints[i].pose.position);
      if (dist < refine_radius_) {
        nearby_indices.push_back(i);
      }
    }
    
    if (nearby_indices.empty()) {
      // No nearby waypoints to refine
      refined_pub_->publish(*msg);
      return;
    }
    
    // Build graph and find optimal viewpoint sequence using Dijkstra
    auto refined_waypoints = refineWithDijkstra(msg, nearby_indices);
    
    // Build output message
    exploration_planner::msg::ExplorationStatus refined = *msg;
    refined.header.stamp = now();
    refined.state = exploration_planner::msg::ExplorationStatus::REFINING;
    
    // Replace nearby waypoints with refined ones
    for (size_t i = 0; i < nearby_indices.size() && i < refined_waypoints.size(); ++i) {
      refined.waypoints[nearby_indices[i]] = refined_waypoints[i];
    }
    
    // Update current target
    if (!refined.waypoints.empty()) {
      refined.current_target = refined.waypoints[refined.current_waypoint_index];
    }
    
    refined_pub_->publish(refined);
    
    auto duration = (now() - start_time).seconds() * 1000.0;
    RCLCPP_DEBUG(get_logger(), "Refined %zu waypoints in %.1f ms", 
                 nearby_indices.size(), duration);
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> refineWithDijkstra(
    const exploration_planner::msg::ExplorationStatus::SharedPtr& tour,
    const std::vector<size_t>& nearby_indices)
  {
    std::vector<geometry_msgs::msg::PoseStamped> result;
    
    if (!current_clusters_ || nearby_indices.empty()) {
      return result;
    }
    
    // Map cluster IDs to cluster data
    std::unordered_map<uint32_t, const frontier_exploration::msg::FrontierCluster*> cluster_map;
    for (const auto& cluster : current_clusters_->clusters) {
      cluster_map[cluster.id] = &cluster;
    }
    
    // Build graph nodes
    // Each node = (cluster_index, viewpoint_index)
    // Plus a start node for current position
    
    struct GraphNode {
      int cluster_idx;  // -1 for start
      int vp_idx;
      double cost;
      int prev_node;
    };
    
    std::vector<GraphNode> nodes;
    
    // Add start node
    nodes.push_back({-1, -1, 0.0, -1});
    
    // Add viewpoint nodes for each nearby cluster
    std::vector<std::vector<int>> cluster_nodes(nearby_indices.size());
    
    for (size_t ci = 0; ci < nearby_indices.size(); ++ci) {
      size_t tour_idx = nearby_indices[ci];
      if (tour_idx >= tour->cluster_order.size()) continue;
      
      uint32_t cluster_id = tour->cluster_order[tour_idx];
      auto it = cluster_map.find(cluster_id);
      if (it == cluster_map.end()) continue;
      
      const auto* cluster = it->second;
      
      for (size_t vi = 0; vi < cluster->viewpoints.size(); ++vi) {
        cluster_nodes[ci].push_back(nodes.size());
        nodes.push_back({static_cast<int>(ci), static_cast<int>(vi), 
                         std::numeric_limits<double>::infinity(), -1});
      }
    }
    
    // Dijkstra
    auto cmp = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
      return a.first > b.first;
    };
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>,
                        decltype(cmp)> pq(cmp);
    
    pq.push({0.0, 0});  // Start node
    
    geometry_msgs::msg::Point current_pos = current_pose_.pose.position;
    double current_yaw = getYaw(current_pose_.pose.orientation);
    geometry_msgs::msg::Point current_vel;
    current_vel.x = current_velocity_.linear.x;
    current_vel.y = current_velocity_.linear.y;
    
    while (!pq.empty()) {
      auto [cost, node_idx] = pq.top();
      pq.pop();
      
      if (cost > nodes[node_idx].cost) continue;
      
      const auto& node = nodes[node_idx];
      
      // Determine next cluster index
      int next_cluster = node.cluster_idx + 1;
      if (next_cluster >= static_cast<int>(nearby_indices.size())) continue;
      
      // Get position/yaw of current node
      geometry_msgs::msg::Point from_pos;
      double from_yaw;
      
      if (node.cluster_idx < 0) {
        from_pos = current_pos;
        from_yaw = current_yaw;
      } else {
        size_t tour_idx = nearby_indices[node.cluster_idx];
        uint32_t cluster_id = tour->cluster_order[tour_idx];
        auto it = cluster_map.find(cluster_id);
        if (it == cluster_map.end()) continue;
        
        const auto& vp = it->second->viewpoints[node.vp_idx];
        from_pos = vp.position;
        from_yaw = vp.yaw;
      }
      
      // Expand to all viewpoints of next cluster
      for (int next_node_idx : cluster_nodes[next_cluster]) {
        const auto& next_node = nodes[next_node_idx];
        
        size_t next_tour_idx = nearby_indices[next_cluster];
        uint32_t next_cluster_id = tour->cluster_order[next_tour_idx];
        auto it = cluster_map.find(next_cluster_id);
        if (it == cluster_map.end()) continue;
        
        const auto& vp = it->second->viewpoints[next_node.vp_idx];
        
        // Compute edge cost
        double t_lb = computeTimeLowerBound(from_pos, from_yaw, vp.position, vp.yaw,
                                            v_max_, yaw_rate_max_);
        
        double cc = 0.0;
        if (node.cluster_idx < 0) {
          cc = computeMotionConsistencyCost(current_pos, current_vel, vp.position);
        }
        
        double edge_cost = t_lb + w_consistency_ * cc;
        double new_cost = cost + edge_cost;
        
        if (new_cost < nodes[next_node_idx].cost) {
          nodes[next_node_idx].cost = new_cost;
          nodes[next_node_idx].prev_node = node_idx;
          pq.push({new_cost, next_node_idx});
        }
      }
    }
    
    // Find best end node (in last cluster)
    if (nearby_indices.empty()) return result;
    
    int last_cluster = nearby_indices.size() - 1;
    double best_cost = std::numeric_limits<double>::infinity();
    int best_end = -1;
    
    for (int node_idx : cluster_nodes[last_cluster]) {
      if (nodes[node_idx].cost < best_cost) {
        best_cost = nodes[node_idx].cost;
        best_end = node_idx;
      }
    }
    
    if (best_end < 0) return result;
    
    // Backtrack to build path
    std::vector<int> path;
    int curr = best_end;
    while (curr > 0) {  // Skip start node
      path.push_back(curr);
      curr = nodes[curr].prev_node;
    }
    std::reverse(path.begin(), path.end());
    
    // Convert to waypoints
    for (int node_idx : path) {
      const auto& node = nodes[node_idx];
      size_t tour_idx = nearby_indices[node.cluster_idx];
      uint32_t cluster_id = tour->cluster_order[tour_idx];
      
      auto it = cluster_map.find(cluster_id);
      if (it == cluster_map.end()) continue;
      
      const auto& vp = it->second->viewpoints[node.vp_idx];
      
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header.stamp = now();
      waypoint.header.frame_id = tour->header.frame_id;
      waypoint.pose.position = vp.position;
      waypoint.pose.orientation = yawToQuaternion(vp.yaw);
      result.push_back(waypoint);
    }
    
    return result;
  }
  
  // Parameters
  double refine_radius_;
  int max_refine_clusters_;
  double v_max_;
  double yaw_rate_max_;
  double w_consistency_;
  
  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  bool have_pose_ = false;
  frontier_exploration::msg::FrontierArray::SharedPtr current_clusters_;
  
  // ROS
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr global_tour_sub_;
  rclcpp::Subscription<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<exploration_planner::msg::ExplorationStatus>::SharedPtr refined_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalRefinerNode>());
  rclcpp::shutdown();
  return 0;
}