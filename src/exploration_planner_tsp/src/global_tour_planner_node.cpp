/**
 * Node 1: Global Tour Planner
 * 
 * Input:  /frontier_clusters_complete (frontier_exploration/FrontierArray)
 *         /odom or /mavros/local_position/pose (current position)
 * Output: /exploration/global_tour (exploration_planner/ExplorationStatus)
 * 
 * Services:
 *   /exploration/start (std_srvs/Trigger) - Start exploration
 *   /exploration/stop (std_srvs/Trigger)  - Stop exploration
 * 
 * Solves TSP to find optimal cluster visit order.
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

class GlobalTourPlannerNode : public rclcpp::Node
{
public:
  GlobalTourPlannerNode() : Node("global_tour_planner")
  {
    // Parameters
    declare_parameter("v_max", 1.0);
    declare_parameter("yaw_rate_max", 1.0);
    declare_parameter("w_consistency", 0.5);
    declare_parameter("use_2opt", true);
    declare_parameter("replan_threshold", 2.0);  // Replan if position changes > this
    
    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    w_consistency_ = get_parameter("w_consistency").as_double();
    use_2opt_ = get_parameter("use_2opt").as_bool();
    replan_threshold_ = get_parameter("replan_threshold").as_double();
    
    // Subscribers
    clusters_sub_ = create_subscription<frontier_exploration::msg::FrontierArray>(
      "/frontier_clusters_complete", 10,
      std::bind(&GlobalTourPlannerNode::clustersCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&GlobalTourPlannerNode::odomCallback, this, std::placeholders::_1));
    
    // Also try pose topic (for PX4)
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      std::bind(&GlobalTourPlannerNode::poseCallback, this, std::placeholders::_1));
    
    // Publisher
    tour_pub_ = create_publisher<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10);
    
    // Services
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/exploration/start",
      std::bind(&GlobalTourPlannerNode::startCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/exploration/stop",
      std::bind(&GlobalTourPlannerNode::stopCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(get_logger(), "Global Tour Planner initialized");
    RCLCPP_INFO(get_logger(), "  v_max: %.2f m/s, yaw_rate_max: %.2f rad/s", v_max_, yaw_rate_max_);
    RCLCPP_INFO(get_logger(), "  w_consistency: %.2f, use_2opt: %s", 
                w_consistency_, use_2opt_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Services: /exploration/start, /exploration/stop");
    RCLCPP_INFO(get_logger(), "  Status: IDLE (call /exploration/start to begin)");
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
    
    // Publish IDLE status
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
    if (!have_pose_) {  // Prefer odom if available
      current_pose_ = *msg;
      have_pose_ = true;
    }
  }
  
  void clustersCallback(const frontier_exploration::msg::FrontierArray::SharedPtr msg)
  {
    if (!exploration_active_) {
      return;  // Don't plan if exploration not started
    }
    
    if (!have_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                           "No pose received yet, waiting...");
      return;
    }
    
    if (msg->clusters.empty()) {
      // No frontiers - exploration complete
      exploration_planner::msg::ExplorationStatus status;
      status.header.stamp = now();
      status.header.frame_id = msg->header.frame_id;
      status.state = exploration_planner::msg::ExplorationStatus::COMPLETED;
      tour_pub_->publish(status);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "No frontiers - exploration complete!");
      return;
    }
    
    auto start_time = now();
    
    // Build cost matrix for TSP
    // Node 0 = current position, Nodes 1..N = clusters
    size_t n = msg->clusters.size() + 1;
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n, 0.0));
    
    // Current position and velocity
    geometry_msgs::msg::Point current_pos = current_pose_.pose.position;
    double current_yaw = getYaw(current_pose_.pose.orientation);
    geometry_msgs::msg::Point current_vel;
    current_vel.x = current_velocity_.linear.x;
    current_vel.y = current_velocity_.linear.y;
    
    // Fill cost matrix
    // Cost from current position (node 0) to each cluster
    for (size_t k = 0; k < msg->clusters.size(); ++k) {
      const auto& cluster = msg->clusters[k];
      if (cluster.viewpoints.empty()) continue;
      
      const auto& vp = cluster.viewpoints[0];  // Best viewpoint
      
      double t_lb = computeTimeLowerBound(
        current_pos, current_yaw,
        vp.position, vp.yaw,
        v_max_, yaw_rate_max_);
      
      double cc = computeMotionConsistencyCost(current_pos, current_vel, vp.position);
      
      cost_matrix[0][k + 1] = t_lb + w_consistency_ * cc;
      cost_matrix[k + 1][0] = 0.0;  // Zero cost back to start (ATSP trick)
    }
    
    // Cost between clusters
    for (size_t i = 0; i < msg->clusters.size(); ++i) {
      for (size_t j = 0; j < msg->clusters.size(); ++j) {
        if (i == j) continue;
        
        const auto& c1 = msg->clusters[i];
        const auto& c2 = msg->clusters[j];
        
        if (c1.viewpoints.empty() || c2.viewpoints.empty()) continue;
        
        // Use precomputed connection cost if available
        double cost = std::numeric_limits<double>::infinity();
        for (const auto& conn : c1.connections) {
          if (conn.target_cluster_id == c2.id) {
            cost = conn.cost;
            break;
          }
        }
        
        // Fallback to direct computation
        if (std::isinf(cost)) {
          cost = computeTimeLowerBound(
            c1.viewpoints[0].position, c1.viewpoints[0].yaw,
            c2.viewpoints[0].position, c2.viewpoints[0].yaw,
            v_max_, yaw_rate_max_);
        }
        
        cost_matrix[i + 1][j + 1] = cost;
      }
    }
    
    // Solve TSP
    auto tour = solveGreedyTSP(cost_matrix, 0);
    
    // Improve with 2-opt
    if (use_2opt_ && tour.size() > 3) {
      improve2Opt(tour, cost_matrix);
    }
    
    // Build output message
    exploration_planner::msg::ExplorationStatus status;
    status.header.stamp = now();
    status.header.frame_id = msg->header.frame_id;
    status.state = exploration_planner::msg::ExplorationStatus::EXPLORING;
    
    // Convert tour to cluster IDs and waypoints
    double total_time = 0.0;
    double total_distance = 0.0;
    
    for (size_t i = 1; i < tour.size(); ++i) {  // Skip node 0 (current position)
      size_t cluster_idx = tour[i] - 1;
      const auto& cluster = msg->clusters[cluster_idx];
      
      status.cluster_order.push_back(cluster.id);
      
      if (!cluster.viewpoints.empty()) {
        geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header = status.header;
        waypoint.pose.position = cluster.viewpoints[0].position;
        waypoint.pose.orientation = yawToQuaternion(cluster.viewpoints[0].yaw);
        status.waypoints.push_back(waypoint);
        
        // Accumulate costs
        if (i > 0) {
          total_time += cost_matrix[tour[i-1]][tour[i]];
          
          geometry_msgs::msg::Point prev_pos;
          if (i == 1) {
            prev_pos = current_pos;
          } else {
            size_t prev_idx = tour[i-1] - 1;
            prev_pos = msg->clusters[prev_idx].viewpoints[0].position;
          }
          total_distance += distance2D(prev_pos, cluster.viewpoints[0].position);
        }
      }
    }
    
    // Set current target
    if (!status.waypoints.empty()) {
      status.current_target = status.waypoints[0];
    }
    
    status.current_waypoint_index = 0;
    status.total_waypoints = status.waypoints.size();
    status.estimated_time_remaining = total_time;
    status.total_distance_remaining = total_distance;
    
    tour_pub_->publish(status);
    
    auto duration = (now() - start_time).seconds() * 1000.0;
    RCLCPP_DEBUG(get_logger(), "TSP solved: %zu waypoints, est. time: %.1fs (computed in %.1fms)",
                 status.waypoints.size(), total_time, duration);
  }
  
  // Parameters
  double v_max_;
  double yaw_rate_max_;
  double w_consistency_;
  bool use_2opt_;
  double replan_threshold_;
  
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
  rclcpp::spin(std::make_shared<GlobalTourPlannerNode>());
  rclcpp::shutdown();
  return 0;
}