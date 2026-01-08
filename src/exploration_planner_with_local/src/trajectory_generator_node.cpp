/**
 * Node 3: Trajectory Generator
 * 
 * Input:  /exploration/planned_path (nav_msgs/Path) - from path planner
 *         /exploration/refined_tour (exploration_planner/ExplorationStatus) - fallback
 *         /odom (current state)
 * Output: /exploration/trajectory (exploration_planner/Trajectory)
 * 
 * Generates smooth trajectory following the planned collision-free path.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "exploration_planner/msg/exploration_status.hpp"
#include "exploration_planner/msg/trajectory.hpp"
#include "exploration_planner/common.hpp"

#include <vector>
#include <cmath>

using namespace exploration_planner;

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode() : Node("trajectory_generator")
  {
    // Dynamic constraints (from shared params)
    declare_parameter("v_max", 4.0);
    declare_parameter("a_max", 1.5);
    declare_parameter("yaw_rate_max", 2.5);
    declare_parameter("yaw_accel_max", 1.5);
    
    // Trajectory parameters
    declare_parameter("traj_dt", 0.1);              // Time step
    declare_parameter("lookahead_time", 3.0);       // Max trajectory duration
    declare_parameter("min_waypoint_dist", 0.2);    // Min distance between waypoints
    
    // Yaw blend parameters - smooth yaw distribution along path
    declare_parameter("yaw_initial_blend_dist", 1.0);  // [m] Distance to blend from current to movement direction
    declare_parameter("yaw_final_blend_dist", 2.5);    // [m] Distance to blend from movement to target direction
    
    v_max_ = get_parameter("v_max").as_double();
    a_max_ = get_parameter("a_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    yaw_accel_max_ = get_parameter("yaw_accel_max").as_double();
    dt_ = get_parameter("traj_dt").as_double();
    lookahead_time_ = get_parameter("lookahead_time").as_double();
    min_waypoint_dist_ = get_parameter("min_waypoint_dist").as_double();
    yaw_initial_blend_dist_ = get_parameter("yaw_initial_blend_dist").as_double();
    yaw_final_blend_dist_ = get_parameter("yaw_final_blend_dist").as_double();
    
    // Subscribers
    tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/refined_tour", 10,
      std::bind(&TrajectoryGeneratorNode::tourCallback, this, std::placeholders::_1));
    
    // Subscribe to both planned_path and adjusted_path
    // If local_path_adjuster is running, it will publish to adjusted_path
    // Otherwise, we use planned_path directly
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/exploration/planned_path", 10,
      std::bind(&TrajectoryGeneratorNode::plannedPathCallback, this, std::placeholders::_1));
    
    adjusted_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/exploration/adjusted_path", 10,
      std::bind(&TrajectoryGeneratorNode::adjustedPathCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    traj_pub_ = create_publisher<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10);
    
    RCLCPP_INFO(get_logger(), "Trajectory Generator initialized");
    RCLCPP_INFO(get_logger(), "  v_max: %.2f, a_max: %.2f", v_max_, a_max_);
    RCLCPP_INFO(get_logger(), "  Yaw blend: initial=%.1fm, final=%.1fm", 
                yaw_initial_blend_dist_, yaw_final_blend_dist_);
    RCLCPP_INFO(get_logger(), "  Listening to both /planned_path and /adjusted_path");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    current_twist_ = msg->twist.twist;
    have_odom_ = true;
  }
  
  void plannedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // Store planned path
    last_planned_path_ = msg;
    last_planned_path_time_ = now();
    
    // If local_path_adjuster is disabled or not running, we won't receive adjusted_path
    // So we use planned_path directly after a short timeout
    // 
    // Logic:
    // - If we've NEVER received adjusted_path, use planned_path immediately
    // - If we've received adjusted_path before, wait a bit for it to arrive
    
    if (!ever_received_adjusted_path_) {
      // local_path_adjuster not running - use planned_path directly
      processPath(msg);
    } else {
      // local_path_adjuster is running - wait for adjusted_path
      // But set a timer in case it doesn't arrive
      auto time_since_adjusted = (now() - last_adjusted_path_time_).seconds();
      if (time_since_adjusted > 0.5) {
        // Timeout - use planned_path
        RCLCPP_DEBUG(get_logger(), "Adjusted path timeout, using planned_path");
        processPath(msg);
      }
      // Otherwise wait for adjusted_path callback
    }
  }
  
  void adjustedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    ever_received_adjusted_path_ = true;
    last_adjusted_path_time_ = now();
    
    // Always prefer adjusted_path when available
    processPath(msg);
  }
  
  void processPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odom received");
      return;
    }
    
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path - no valid route found");
      return;
    }
    
    // Generate trajectory following the planned path
    auto trajectory = generateTrajectoryFromPath(*msg);
    
    trajectory.header.stamp = now();
    trajectory.header.frame_id = msg->header.frame_id;
    
    traj_pub_->publish(trajectory);
    
    RCLCPP_DEBUG(get_logger(), "Generated trajectory from path: %zu points", 
                 trajectory.poses.size());
  }
  
  void tourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    // This is now only used as fallback if no path planner is running
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odom received");
      return;
    }
    
    if (msg->waypoints.empty() || 
        msg->state == exploration_planner::msg::ExplorationStatus::COMPLETED) {
      return;
    }
    
    // Only use direct trajectory if path planner didn't provide a path
    // (trajectory follower will use the most recent trajectory)
  }
  
  exploration_planner::msg::Trajectory generateTrajectoryFromPath(
    const nav_msgs::msg::Path& path)
  {
    exploration_planner::msg::Trajectory traj;
    
    if (path.poses.empty()) return traj;
    
    // ═══════════════════════════════════════════════════════════════════
    // SMOOTH YAW DISTRIBUTION ALONG PATH
    // ═══════════════════════════════════════════════════════════════════
    // İlk X m: current_yaw → movement_direction (path yönüne dön)
    // Orta:    movement_direction (path boyunca düz git)
    // Son Y m: movement_direction → target_yaw (frontier'a dön)
    // ═══════════════════════════════════════════════════════════════════
    
    // Calculate total path length
    double total_path_length = 0.0;
    std::vector<double> cumulative_dist;
    cumulative_dist.push_back(0.0);
    
    for (size_t i = 1; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
      total_path_length += std::sqrt(dx*dx + dy*dy);
      cumulative_dist.push_back(total_path_length);
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // ADAPTIVE BLEND DISTANCES - Handle short paths
    // ═══════════════════════════════════════════════════════════════════
    double initial_blend = yaw_initial_blend_dist_;
    double final_blend = yaw_final_blend_dist_;
    
    // If path is shorter than both blend zones combined, scale them proportionally
    if (total_path_length < initial_blend + final_blend) {
      if (total_path_length < 0.5) {
        // Very short path - just use target yaw directly
        initial_blend = 0.0;
        final_blend = total_path_length;
      } else {
        // Scale blend distances proportionally
        double scale = total_path_length / (initial_blend + final_blend);
        initial_blend *= scale * 0.3;  // 30% for initial turn
        final_blend *= scale * 0.7;    // 70% for final turn (more important)
      }
      
      RCLCPP_DEBUG(get_logger(), "Short path (%.2fm) - adjusted blends: initial=%.2f, final=%.2f",
                   total_path_length, initial_blend, final_blend);
    }
    
    // Get initial yaw (current drone orientation)
    double initial_yaw = getYaw(current_pose_.orientation);
    
    // Get target yaw (from global_tour_planner - where to look at frontier)
    double target_yaw = getYaw(path.poses.back().pose.orientation);
    
    // Calculate overall movement direction (start to end)
    double overall_dx = path.poses.back().pose.position.x - path.poses.front().pose.position.x;
    double overall_dy = path.poses.back().pose.position.y - path.poses.front().pose.position.y;
    double movement_yaw = std::atan2(overall_dy, overall_dx);
    
    RCLCPP_DEBUG(get_logger(), "Yaw planning: initial=%.1f°, movement=%.1f°, target=%.1f°, path_len=%.2fm",
                 initial_yaw * 180.0 / M_PI, movement_yaw * 180.0 / M_PI, 
                 target_yaw * 180.0 / M_PI, total_path_length);
    
    double total_time = 0.0;
    geometry_msgs::msg::Pose prev_pose = current_pose_;
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto& waypoint = path.poses[i];
      
      // Calculate distance and time to this waypoint
      double dx = waypoint.pose.position.x - prev_pose.position.x;
      double dy = waypoint.pose.position.y - prev_pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      double segment_time = computeSegmentTime(dist);
      
      // Current position along path
      double dist_from_start = cumulative_dist[i];
      double dist_to_end = total_path_length - dist_from_start;
      
      // ═══════════════════════════════════════════════════════════════════
      // CALCULATE YAW based on position along path
      // ═══════════════════════════════════════════════════════════════════
      double waypoint_yaw;
      
      // Check for overlapping zones (when initial_blend + final_blend >= total_path_length)
      bool in_initial_zone = (dist_from_start < initial_blend) && (initial_blend > 0.01);
      bool in_final_zone = (dist_to_end < final_blend) && (final_blend > 0.01);
      
      if (in_initial_zone && in_final_zone) {
        // OVERLAPPING ZONE: Blend directly from initial to target
        double blend = dist_from_start / total_path_length;
        // Use smooth step for nicer transition
        blend = blend * blend * (3.0 - 2.0 * blend);  // smoothstep
        double yaw_diff = normalizeAngle(target_yaw - initial_yaw);
        waypoint_yaw = normalizeAngle(initial_yaw + blend * yaw_diff);
        
      } else if (in_initial_zone) {
        // INITIAL PHASE: Blend from current_yaw to movement_yaw
        double blend = dist_from_start / initial_blend;
        blend = blend * blend * (3.0 - 2.0 * blend);  // smoothstep
        double yaw_diff = normalizeAngle(movement_yaw - initial_yaw);
        waypoint_yaw = normalizeAngle(initial_yaw + blend * yaw_diff);
        
        RCLCPP_DEBUG(get_logger(), "Initial blend: dist=%.2f, blend=%.2f, yaw=%.1f°",
                     dist_from_start, blend, waypoint_yaw * 180.0 / M_PI);
                     
      } else if (in_final_zone) {
        // FINAL PHASE: Blend from movement_yaw to target_yaw
        double blend = 1.0 - (dist_to_end / final_blend);
        blend = blend * blend * (3.0 - 2.0 * blend);  // smoothstep
        double yaw_diff = normalizeAngle(target_yaw - movement_yaw);
        waypoint_yaw = normalizeAngle(movement_yaw + blend * yaw_diff);
        
        RCLCPP_DEBUG(get_logger(), "Final blend: dist_to_end=%.2f, blend=%.2f, yaw=%.1f°",
                     dist_to_end, blend, waypoint_yaw * 180.0 / M_PI);
                     
      } else {
        // MIDDLE PHASE: Face movement direction
        if (i < path.poses.size() - 1) {
          double next_dx = path.poses[i+1].pose.position.x - waypoint.pose.position.x;
          double next_dy = path.poses[i+1].pose.position.y - waypoint.pose.position.y;
          waypoint_yaw = std::atan2(next_dy, next_dx);
        } else {
          waypoint_yaw = movement_yaw;
        }
      }
      
      // Build trajectory pose
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = waypoint.pose;
      pose.pose.orientation = yawToQuaternion(waypoint_yaw);
      
      traj.poses.push_back(pose);
      
      // Calculate velocity at this point
      geometry_msgs::msg::Twist vel;
      if (dist > 0.01) {
        double speed = std::min(v_max_, dist / segment_time);
        vel.linear.x = speed * dx / dist;
        vel.linear.y = speed * dy / dist;
      }
      traj.velocities.push_back(vel);
      
      total_time += segment_time;
      traj.time_from_start.push_back(total_time);
      
      prev_pose = waypoint.pose;
    }
    
    traj.total_time = total_time;
    
    return traj;
  }
  
  double computeSegmentTime(double distance)
  {
    if (distance < 0.01) return 0.1;
    
    // Trapezoidal profile time
    double t_accel = v_max_ / a_max_;
    double d_accel = 0.5 * a_max_ * t_accel * t_accel;
    
    if (2 * d_accel >= distance) {
      // Triangle profile
      return 2.0 * std::sqrt(distance / a_max_);
    } else {
      // Trapezoidal profile
      return 2.0 * t_accel + (distance - 2.0 * d_accel) / v_max_;
    }
  }
  
  exploration_planner::msg::Trajectory generateTrajectory(
    const geometry_msgs::msg::PoseStamped& target)
  {
    exploration_planner::msg::Trajectory traj;
    
    // Current state
    double x0 = current_pose_.position.x;
    double y0 = current_pose_.position.y;
    double yaw0 = getYaw(current_pose_.orientation);
    
    // Current velocities - reserved for future smooth trajectory blending
    // Currently using simple interpolation, but these could be used for:
    // - Smooth acceleration from current velocity
    // - Jerk-limited trajectory generation
    (void)current_twist_;  // Suppress unused warning
    
    // Target state
    double xf = target.pose.position.x;
    double yf = target.pose.position.y;
    double yawf = getYaw(target.pose.orientation);
    
    // Simple minimum-time trajectory using trapezoidal velocity profile
    double dx = xf - x0;
    double dy = yf - y0;
    double dist = std::sqrt(dx*dx + dy*dy);
    double dyaw = normalizeAngle(yawf - yaw0);
    
    // Time estimates
    double t_pos = computeTrapezoidalTime(dist, v_max_, a_max_);
    double t_yaw = computeTrapezoidalTime(std::abs(dyaw), yaw_rate_max_, yaw_accel_max_);
    double total_time = std::max(t_pos, t_yaw);
    total_time = std::min(total_time, lookahead_time_);
    
    if (total_time < dt_) {
      // Already at target
      geometry_msgs::msg::PoseStamped pose;
      pose.header = target.header;
      pose.pose = current_pose_;
      traj.poses.push_back(pose);
      
      geometry_msgs::msg::Twist vel;
      traj.velocities.push_back(vel);
      traj.time_from_start.push_back(0.0);
      traj.total_time = 0.0;
      return traj;
    }
    
    // Generate trajectory points
    int num_points = static_cast<int>(total_time / dt_) + 1;
    
    for (int i = 0; i <= num_points; ++i) {
      double t = i * dt_;
      if (t > total_time) t = total_time;
      
      double s = t / total_time;  // Normalized time [0, 1]
      
      // Smooth interpolation using cubic ease-in-out
      double alpha = smoothStep(s);
      double alpha_dot = smoothStepDerivative(s) / total_time;
      
      // Position
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
      pose.pose.position.x = x0 + alpha * dx;
      pose.pose.position.y = y0 + alpha * dy;
      pose.pose.position.z = target.pose.position.z;
      
      // Yaw (interpolate shortest path)
      double yaw = yaw0 + alpha * dyaw;
      pose.pose.orientation = yawToQuaternion(yaw);
      
      traj.poses.push_back(pose);
      
      // Velocity
      geometry_msgs::msg::Twist vel;
      vel.linear.x = alpha_dot * dx;
      vel.linear.y = alpha_dot * dy;
      vel.angular.z = alpha_dot * dyaw;
      
      // Clamp velocities
      double v_linear = std::sqrt(vel.linear.x*vel.linear.x + vel.linear.y*vel.linear.y);
      if (v_linear > v_max_) {
        double scale = v_max_ / v_linear;
        vel.linear.x *= scale;
        vel.linear.y *= scale;
      }
      vel.angular.z = std::max(-yaw_rate_max_, std::min(yaw_rate_max_, vel.angular.z));
      
      traj.velocities.push_back(vel);
      traj.time_from_start.push_back(t);
    }
    
    traj.total_time = total_time;
    return traj;
  }
  
  // Trapezoidal profile time calculation
  double computeTrapezoidalTime(double dist, double v_max, double a_max)
  {
    // Time to accelerate to v_max
    double t_accel = v_max / a_max;
    double d_accel = 0.5 * a_max * t_accel * t_accel;
    
    if (2 * d_accel >= dist) {
      // Triangular profile (can't reach v_max)
      return 2.0 * std::sqrt(dist / a_max);
    } else {
      // Trapezoidal profile
      double d_cruise = dist - 2 * d_accel;
      double t_cruise = d_cruise / v_max;
      return 2 * t_accel + t_cruise;
    }
  }
  
  // Smooth step function (cubic ease-in-out)
  double smoothStep(double t)
  {
    if (t <= 0) return 0;
    if (t >= 1) return 1;
    return t * t * (3 - 2 * t);
  }
  
  double smoothStepDerivative(double t)
  {
    if (t <= 0 || t >= 1) return 0;
    return 6 * t * (1 - t);
  }
  
  // Parameters
  double v_max_, a_max_;
  double yaw_rate_max_, yaw_accel_max_;
  double dt_, lookahead_time_;
  double min_waypoint_dist_;
  double yaw_initial_blend_dist_;   // Distance to blend from current to movement yaw
  double yaw_final_blend_dist_;     // Distance to blend from movement to target yaw
  
  // State
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_twist_;
  bool have_odom_ = false;
  
  // Path timing for choosing between planned and adjusted
  rclcpp::Time last_planned_path_time_;
  rclcpp::Time last_adjusted_path_time_;
  nav_msgs::msg::Path::SharedPtr last_planned_path_;
  bool ever_received_adjusted_path_ = false;  // Track if local_path_adjuster is running
  
  // ROS
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr tour_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr adjusted_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<exploration_planner::msg::Trajectory>::SharedPtr traj_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}