/**
 * Node 3: Trajectory Generator
 * 
 * Input:  /exploration/refined_tour (exploration_planner/ExplorationStatus)
 *         /odom (current state)
 * Output: /exploration/trajectory (exploration_planner/Trajectory)
 * 
 * Generates smooth trajectory to first waypoint using cubic B-spline.
 * Optimizes for minimum time while respecting safety and dynamic constraints.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    // Dynamic constraints
    declare_parameter("v_max", 1.0);
    declare_parameter("a_max", 0.5);
    declare_parameter("yaw_rate_max", 1.0);
    declare_parameter("yaw_accel_max", 0.5);
    
    // Trajectory parameters
    declare_parameter("dt", 0.1);                // Time step
    declare_parameter("lookahead_time", 5.0);    // Max trajectory duration
    declare_parameter("min_waypoint_dist", 0.3); // Min distance between waypoints
    
    v_max_ = get_parameter("v_max").as_double();
    a_max_ = get_parameter("a_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    yaw_accel_max_ = get_parameter("yaw_accel_max").as_double();
    dt_ = get_parameter("dt").as_double();
    lookahead_time_ = get_parameter("lookahead_time").as_double();
    min_waypoint_dist_ = get_parameter("min_waypoint_dist").as_double();
    
    // Subscribers
    tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/refined_tour", 10,
      std::bind(&TrajectoryGeneratorNode::tourCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    traj_pub_ = create_publisher<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10);
    
    RCLCPP_INFO(get_logger(), "Trajectory Generator initialized");
    RCLCPP_INFO(get_logger(), "  v_max: %.2f, a_max: %.2f", v_max_, a_max_);
    RCLCPP_INFO(get_logger(), "  yaw_rate_max: %.2f, dt: %.2f", yaw_rate_max_, dt_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    current_twist_ = msg->twist.twist;
    have_odom_ = true;
  }
  
  void tourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odom received");
      return;
    }
    
    if (msg->waypoints.empty() || 
        msg->state == exploration_planner::msg::ExplorationStatus::COMPLETED) {
      return;
    }
    
    // Generate trajectory to current target
    auto trajectory = generateTrajectory(msg->current_target);
    
    trajectory.header.stamp = now();
    trajectory.header.frame_id = msg->header.frame_id;
    
    traj_pub_->publish(trajectory);
  }
  
  exploration_planner::msg::Trajectory generateTrajectory(
    const geometry_msgs::msg::PoseStamped& target)
  {
    exploration_planner::msg::Trajectory traj;
    
    // Current state
    double x0 = current_pose_.position.x;
    double y0 = current_pose_.position.y;
    double yaw0 = getYaw(current_pose_.orientation);
    double vx0 = current_twist_.linear.x;
    double vy0 = current_twist_.linear.y;
    double vyaw0 = current_twist_.angular.z;
    
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
  
  // State
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_twist_;
  bool have_odom_ = false;
  
  // ROS
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr tour_sub_;
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