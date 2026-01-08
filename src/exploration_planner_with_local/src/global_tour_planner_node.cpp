/**
 * Node 1: Greedy Frontier Selector
 * 
 * Input:  /frontier_clusters_complete (frontier_exploration/FrontierArray)
 *         /odom or /mavros/local_position/pose (current position)
 * Output: /exploration/global_tour (exploration_planner/ExplorationStatus)
 * 
 * Services:
 *   /exploration/start (std_srvs/Trigger) - Start exploration
 *   /exploration/stop (std_srvs/Trigger)  - Stop exploration
 * 
 * Selects best frontier using weighted cost function:
 *   cost = w_dist * distance + w_size * (1/cluster_size) + w_angle * angle_change
 * 
 * IMPROVED: Evaluates ALL viewpoints (top N per cluster), not just the best one!
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "frontier_exploration/msg/frontier_array.hpp"
#include "exploration_planner/msg/exploration_status.hpp"
#include "exploration_planner/common.hpp"

using namespace exploration_planner;

class GreedyFrontierSelectorNode : public rclcpp::Node
{
public:
  GreedyFrontierSelectorNode() : Node("greedy_frontier_selector")
  {
    // Cost function weights
    declare_parameter("w_distance", 1.0);
    declare_parameter("w_size", 0.3);
    declare_parameter("w_angle", 0.5);
    declare_parameter("w_coverage", 0.6);
    
    // Viewpoint selection
    declare_parameter("max_viewpoints_to_evaluate", 5);  // Evaluate top N viewpoints per cluster
    
    // Motion limits
    declare_parameter("v_max", 4.0);
    declare_parameter("yaw_rate_max", 2.5);
    
    w_distance_ = get_parameter("w_distance").as_double();
    w_size_ = get_parameter("w_size").as_double();
    w_angle_ = get_parameter("w_angle").as_double();
    w_coverage_ = get_parameter("w_coverage").as_double();
    max_viewpoints_to_evaluate_ = get_parameter("max_viewpoints_to_evaluate").as_int();
    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    
    // Subscribers
    clusters_sub_ = create_subscription<frontier_exploration::msg::FrontierArray>(
      "/frontier_clusters_complete", 10,
      std::bind(&GreedyFrontierSelectorNode::clustersCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&GreedyFrontierSelectorNode::odomCallback, this, std::placeholders::_1));
    
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      std::bind(&GreedyFrontierSelectorNode::poseCallback, this, std::placeholders::_1));
    
    // Publisher
    tour_pub_ = create_publisher<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10);
    
    // Services
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/exploration/start",
      std::bind(&GreedyFrontierSelectorNode::startCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/exploration/stop",
      std::bind(&GreedyFrontierSelectorNode::stopCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(get_logger(), "Greedy Frontier Selector initialized");
    RCLCPP_INFO(get_logger(), "  Cost weights: dist=%.2f, size=%.2f, angle=%.2f, coverage=%.2f",
                w_distance_, w_size_, w_angle_, w_coverage_);
    RCLCPP_INFO(get_logger(), "  Max viewpoints to evaluate: %d", max_viewpoints_to_evaluate_);
  }

private:
  void startCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    exploration_active_ = true;
    response->success = true;
    response->message = "Exploration started";
    RCLCPP_INFO(get_logger(), "Exploration STARTED");
  }
  
  void stopCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    exploration_active_ = false;
    
    exploration_planner::msg::ExplorationStatus status;
    status.header.stamp = now();
    status.state = exploration_planner::msg::ExplorationStatus::IDLE;
    tour_pub_->publish(status);
    
    response->success = true;
    response->message = "Exploration stopped";
    RCLCPP_INFO(get_logger(), "Exploration STOPPED");
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    current_velocity_ = msg->twist.twist;
    have_pose_ = true;
  }
  
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!have_pose_) {
      current_pose_ = *msg;
      have_pose_ = true;
    }
  }
  
  void clustersCallback(const frontier_exploration::msg::FrontierArray::SharedPtr msg)
  {
    if (!exploration_active_) return;
    
    if (!have_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No pose received yet");
      return;
    }
    
    // Filter clusters - only keep those with valid viewpoints
    std::vector<const frontier_exploration::msg::FrontierCluster*> valid_clusters;
    for (const auto& cluster : msg->clusters) {
      if (!cluster.viewpoints.empty()) {
        valid_clusters.push_back(&cluster);
      }
    }
    
    if (valid_clusters.empty()) {
      exploration_planner::msg::ExplorationStatus status;
      status.header.stamp = now();
      status.header.frame_id = msg->header.frame_id;
      status.state = exploration_planner::msg::ExplorationStatus::COMPLETED;
      tour_pub_->publish(status);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "No valid frontiers - exploration complete!");
      return;
    }
    
    // Find best viewpoint across ALL clusters
    auto [best_cluster, best_vp_idx, best_cost] = selectBestViewpoint(valid_clusters);
    
    if (!best_cluster) {
      RCLCPP_WARN(get_logger(), "Could not select a valid viewpoint");
      return;
    }
    
    const auto& vp = best_cluster->viewpoints[best_vp_idx];
    
    // Build output
    exploration_planner::msg::ExplorationStatus status;
    status.header.stamp = now();
    status.header.frame_id = msg->header.frame_id;
    status.state = exploration_planner::msg::ExplorationStatus::EXPLORING;
    
    status.cluster_order.push_back(best_cluster->id);
    
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = status.header;
    waypoint.pose.position = vp.position;
    
    // ═══════════════════════════════════════════════════════════════════
    // Calculate target yaw - where camera should look when arriving
    // vp.yaw from frontier_exploration is often 0, so we calculate it ourselves
    // Yaw = direction from viewpoint to cluster centroid (frontier)
    // ═══════════════════════════════════════════════════════════════════
    double cx = best_cluster->centroid.x;
    double cy = best_cluster->centroid.y;
    
    // Calculate yaw from viewpoint position to cluster centroid
    double target_yaw = std::atan2(cy - vp.position.y, cx - vp.position.x);
    
    waypoint.pose.orientation = yawToQuaternion(target_yaw);
    
    status.waypoints.push_back(waypoint);
    status.current_target = waypoint;
    status.current_waypoint_index = 0;
    status.total_waypoints = 1;
    
    // Estimate time
    double dist = distance2D(current_pose_.pose.position, waypoint.pose.position);
    status.estimated_time_remaining = dist / v_max_;
    status.total_distance_remaining = dist;
    
    tour_pub_->publish(status);
    
    RCLCPP_INFO(get_logger(), "Selected cluster %d vp[%d], dist=%.2f, cost=%.3f, cov=%d, yaw=%.1f°", 
                best_cluster->id, best_vp_idx, dist, best_cost, vp.coverage, target_yaw * 180.0 / M_PI);
  }
  
  /**
   * Evaluate ALL viewpoints from ALL clusters and return the best one
   * Returns: (cluster_ptr, viewpoint_index, cost)
   */
  std::tuple<const frontier_exploration::msg::FrontierCluster*, int, double>
  selectBestViewpoint(const std::vector<const frontier_exploration::msg::FrontierCluster*>& clusters)
  {
    const frontier_exploration::msg::FrontierCluster* best_cluster = nullptr;
    int best_vp_idx = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    
    geometry_msgs::msg::Point current_pos = current_pose_.pose.position;
    double current_yaw = getYaw(current_pose_.pose.orientation);
    
    // Get current velocity direction
    double vel_angle = std::atan2(current_velocity_.linear.y, current_velocity_.linear.x);
    double vel_mag = std::sqrt(
      current_velocity_.linear.x * current_velocity_.linear.x +
      current_velocity_.linear.y * current_velocity_.linear.y);
    
    // Find max values for normalization
    uint32_t max_size = 1;
    int max_coverage = 1;
    double max_dist = 1.0;
    
    for (const auto* cluster : clusters) {
      max_size = std::max(max_size, cluster->size);
      for (const auto& vp : cluster->viewpoints) {
        max_coverage = std::max(max_coverage, vp.coverage);
        double dist = distance2D(current_pos, vp.position);
        max_dist = std::max(max_dist, dist);
      }
    }
    
    // Evaluate each cluster's viewpoints
    for (const auto* cluster : clusters) {
      int num_vps = std::min(static_cast<int>(cluster->viewpoints.size()), 
                             max_viewpoints_to_evaluate_);
      
      for (int vp_idx = 0; vp_idx < num_vps; ++vp_idx) {
        const auto& vp = cluster->viewpoints[vp_idx];
        
        // === Cost Components ===
        
        // 1. Distance cost (normalized)
        double dist = distance2D(current_pos, vp.position);
        double dist_cost = dist / max_dist;
        
        // 2. Size cost (inverted - bigger is better)
        double size_cost = 1.0 - (static_cast<double>(cluster->size) / max_size);
        
        // 3. Angle change cost
        double angle_to_target = std::atan2(
          vp.position.y - current_pos.y,
          vp.position.x - current_pos.x);
        double angle_cost = 0.0;
        if (vel_mag > 0.1) {
          angle_cost = angleDiff(vel_angle, angle_to_target) / M_PI;
        } else {
          angle_cost = angleDiff(current_yaw, angle_to_target) / M_PI;
        }
        
        // 4. Coverage cost (inverted - higher coverage is better)
        double coverage_cost = 1.0 - (static_cast<double>(vp.coverage) / max_coverage);
        
        // === Total Cost ===
        double total_cost = w_distance_ * dist_cost +
                            w_size_ * size_cost +
                            w_angle_ * angle_cost +
                            w_coverage_ * coverage_cost;
        
        RCLCPP_DEBUG(get_logger(), 
                     "Cluster %d VP[%d]: dist=%.2f(%.2f), size=%.2f, angle=%.2f, cov=%.2f -> cost=%.3f",
                     cluster->id, vp_idx, dist, dist_cost, size_cost, angle_cost, coverage_cost, total_cost);
        
        if (total_cost < best_cost) {
          best_cost = total_cost;
          best_cluster = cluster;
          best_vp_idx = vp_idx;
        }
      }
    }
    
    return {best_cluster, best_vp_idx, best_cost};
  }
  
  // Parameters
  double w_distance_;
  double w_size_;
  double w_angle_;
  double w_coverage_;
  int max_viewpoints_to_evaluate_;
  double v_max_;
  double yaw_rate_max_;
  
  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  bool have_pose_ = false;
  bool exploration_active_ = false;
  
  // ROS
  rclcpp::Subscription<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<exploration_planner::msg::ExplorationStatus>::SharedPtr tour_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreedyFrontierSelectorNode>());
  rclcpp::shutdown();
  return 0;
}