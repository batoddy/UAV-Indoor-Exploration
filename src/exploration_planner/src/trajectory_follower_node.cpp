/**
 * Node 4: Trajectory Follower
 * 
 * Input:  /exploration/trajectory (exploration_planner/Trajectory)
 *         /odom (current state)
 * Output: /cmd_vel (geometry_msgs/Twist)
 * 
 * SMOOTH MOTION with:
 * - Velocity rate limiting (acceleration control)
 * - Exponential smoothing
 * - Gentle approach to targets
 * - PROGRESSIVE YAW: Start turning towards viewpoint before arrival
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "exploration_planner/msg/trajectory.hpp"
#include "exploration_planner/common.hpp"

using namespace exploration_planner;

class TrajectoryFollowerNode : public rclcpp::Node
{
public:
  TrajectoryFollowerNode() : Node("trajectory_follower")
  {
    // Control parameters
    declare_parameter("control_rate", 30.0);
    declare_parameter("goal_tolerance_xy", 0.25);
    declare_parameter("goal_tolerance_yaw", 0.15);
    
    // Gains
    declare_parameter("kp_linear", 0.8);
    declare_parameter("kp_yaw", 1.2);
    declare_parameter("kp_z", 1.0);  // Height control gain
    
    // Velocity limits
    declare_parameter("v_max", 2.5);
    declare_parameter("yaw_rate_max", 1.2);
    declare_parameter("vz_max", 1.0);  // Max vertical velocity
    
    // Acceleration limits
    declare_parameter("a_max", 0.5);
    declare_parameter("yaw_accel_max", 0.8);
    
    // Smoothing
    declare_parameter("velocity_smoothing", 0.3);
    
    // Slowdown
    declare_parameter("slowdown_distance", 2.0);
    declare_parameter("min_approach_speed", 0.3);
    
    // Progressive yaw
    declare_parameter("progressive_yaw_distance", 2.5);
    declare_parameter("progressive_yaw_blend", 0.7);
    
    // Height control
    declare_parameter("target_height", 1.5);       // Desired flight height
    declare_parameter("height_tolerance", 0.2);    // Acceptable height error
    
    control_rate_ = get_parameter("control_rate").as_double();
    goal_tolerance_xy_ = get_parameter("goal_tolerance_xy").as_double();
    goal_tolerance_yaw_ = get_parameter("goal_tolerance_yaw").as_double();
    kp_linear_ = get_parameter("kp_linear").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    kp_z_ = get_parameter("kp_z").as_double();
    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    vz_max_ = get_parameter("vz_max").as_double();
    a_max_ = get_parameter("a_max").as_double();
    yaw_accel_max_ = get_parameter("yaw_accel_max").as_double();
    velocity_smoothing_ = get_parameter("velocity_smoothing").as_double();
    slowdown_distance_ = get_parameter("slowdown_distance").as_double();
    min_approach_speed_ = get_parameter("min_approach_speed").as_double();
    progressive_yaw_distance_ = get_parameter("progressive_yaw_distance").as_double();
    progressive_yaw_blend_ = get_parameter("progressive_yaw_blend").as_double();
    target_height_ = get_parameter("target_height").as_double();
    height_tolerance_ = get_parameter("height_tolerance").as_double();
    
    dt_ = 1.0 / control_rate_;
    
    // Subscribers
    traj_sub_ = create_subscription<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10,
      std::bind(&TrajectoryFollowerNode::trajectoryCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryFollowerNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Control timer
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&TrajectoryFollowerNode::controlLoop, this));
    
    RCLCPP_INFO(get_logger(), "Trajectory Follower initialized (SMOOTH MOTION)");
    RCLCPP_INFO(get_logger(), "  v_max: %.1f m/s, a_max: %.1f m/s²", v_max_, a_max_);
    RCLCPP_INFO(get_logger(), "  yaw_rate_max: %.1f rad/s, smoothing: %.1f", 
                yaw_rate_max_, velocity_smoothing_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    current_twist_ = msg->twist.twist;
    have_odom_ = true;
  }
  
  void trajectoryCallback(const exploration_planner::msg::Trajectory::SharedPtr msg)
  {
    current_trajectory_ = msg;
    have_trajectory_ = true;
    
    if (!msg->poses.empty()) {
      target_pose_ = msg->poses.back().pose;
      have_target_ = true;
      
      double target_yaw = getYaw(target_pose_.orientation);
      RCLCPP_INFO(get_logger(), "New target: pos=(%.2f, %.2f), yaw=%.1f°", 
                   target_pose_.position.x, target_pose_.position.y,
                   target_yaw * 180.0 / M_PI);
    }
  }
  
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;
    
    if (!have_odom_ || !have_target_) {
      cmd = smoothStop();
      cmd_pub_->publish(cmd);
      return;
    }
    
    // Calculate error to target
    double dx = target_pose_.position.x - current_pose_.position.x;
    double dy = target_pose_.position.y - current_pose_.position.y;
    double dist_to_target = std::sqrt(dx*dx + dy*dy);
    
    // Angle to target (movement direction)
    double angle_to_target = std::atan2(dy, dx);
    double current_yaw = getYaw(current_pose_.orientation);
    
    // Target yaw from viewpoint (where camera should look)
    double target_yaw = getYaw(target_pose_.orientation);
    
    // ═══════════════════════════════════════════════════════════════════
    // PROGRESSIVE YAW: Blend between movement direction and target yaw
    // ═══════════════════════════════════════════════════════════════════
    double desired_yaw;
    if (dist_to_target < goal_tolerance_xy_) {
      // At goal: face target yaw
      desired_yaw = target_yaw;
    } else if (dist_to_target < progressive_yaw_distance_) {
      // Approaching: progressively blend to target yaw
      double blend = 1.0 - (dist_to_target / progressive_yaw_distance_);
      blend = blend * progressive_yaw_blend_;
      
      double angle_diff = normalizeAngle(target_yaw - angle_to_target);
      desired_yaw = normalizeAngle(angle_to_target + blend * angle_diff);
      
      RCLCPP_DEBUG(get_logger(), "Progressive yaw: dist=%.2f, blend=%.2f, desired=%.2f°",
                   dist_to_target, blend, desired_yaw * 180.0 / M_PI);
    } else {
      // Far away: face movement direction
      desired_yaw = angle_to_target;
    }
    
    double yaw_error = normalizeAngle(desired_yaw - current_yaw);
    
    // Check if at goal (XY position)
    if (dist_to_target < goal_tolerance_xy_) {
      double final_yaw_error = normalizeAngle(target_yaw - current_yaw);
      
      // Height control - always active
      double current_z = current_pose_.position.z;
      double z_error = target_height_ - current_z;
      double vz_cmd = 0.0;
      if (std::abs(z_error) > height_tolerance_) {
        vz_cmd = clamp(kp_z_ * z_error, -vz_max_, vz_max_);
      }
      
      if (std::abs(final_yaw_error) < goal_tolerance_yaw_) {
        // Fully at goal - stop XY but maintain height
        cmd = smoothStop();
        cmd.linear.z = vz_cmd;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                             "At goal! (dist=%.2fm, yaw_err=%.1f°)", 
                             dist_to_target, final_yaw_error * 180.0 / M_PI);
        return;
      } else {
        // At XY goal but need to rotate
        double desired_yaw_rate = kp_yaw_ * final_yaw_error;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = vz_cmd;  // Height control
        cmd.angular.z = rateLimitYaw(desired_yaw_rate);
        cmd = smoothVelocity(cmd);
        cmd_pub_->publish(cmd);
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, 
                             "At position, rotating: yaw_err=%.1f°, target_yaw=%.1f°, current_yaw=%.1f°, cmd_yaw=%.2f", 
                             final_yaw_error * 180.0 / M_PI,
                             target_yaw * 180.0 / M_PI,
                             current_yaw * 180.0 / M_PI,
                             cmd.angular.z);
        return;
      }
    }
    
    // Calculate desired velocities
    double cos_yaw = std::cos(current_yaw);
    double sin_yaw = std::sin(current_yaw);
    double dx_body = cos_yaw * dx + sin_yaw * dy;
    double dy_body = -sin_yaw * dx + cos_yaw * dy;
    
    // Speed scaling based on distance (slow down near target)
    double speed_scale = 1.0;
    if (dist_to_target < slowdown_distance_) {
      speed_scale = std::max(min_approach_speed_ / v_max_, 
                             dist_to_target / slowdown_distance_);
    }
    
    // Also slow down if not facing target (large yaw error)
    double yaw_factor = std::cos(yaw_error);
    yaw_factor = std::max(0.2, yaw_factor);
    speed_scale *= yaw_factor;
    
    // Desired velocities (X, Y)
    double desired_vx = kp_linear_ * dx_body * speed_scale;
    double desired_vy = kp_linear_ * dy_body * speed_scale;
    double desired_yaw_rate = kp_yaw_ * yaw_error;
    
    // ═══════════════════════════════════════════════════════════════════
    // HEIGHT CONTROL: Maintain target flight height
    // ═══════════════════════════════════════════════════════════════════
    double current_z = current_pose_.position.z;
    double z_error = target_height_ - current_z;
    double desired_vz = 0.0;
    
    if (std::abs(z_error) > height_tolerance_) {
      desired_vz = kp_z_ * z_error;
      desired_vz = clamp(desired_vz, -vz_max_, vz_max_);
      
      RCLCPP_DEBUG(get_logger(), "Height control: current=%.2f, target=%.2f, vz=%.2f",
                   current_z, target_height_, desired_vz);
    }
    
    // Clamp desired velocities
    double v_linear = std::sqrt(desired_vx*desired_vx + desired_vy*desired_vy);
    double effective_v_max = v_max_ * speed_scale;
    
    if (v_linear > effective_v_max) {
      double scale = effective_v_max / v_linear;
      desired_vx *= scale;
      desired_vy *= scale;
    }
    
    // Apply rate limiting (acceleration control)
    cmd.linear.x = rateLimitLinear(desired_vx, last_cmd_.linear.x);
    cmd.linear.y = rateLimitLinear(desired_vy, last_cmd_.linear.y);
    cmd.linear.z = rateLimitLinear(desired_vz, last_cmd_.linear.z);  // Height control
    cmd.angular.z = rateLimitYaw(desired_yaw_rate);
    
    // Apply exponential smoothing
    cmd = smoothVelocity(cmd);
    
    cmd_pub_->publish(cmd);
    last_cmd_ = cmd;
  }
  
  double rateLimitLinear(double desired, double current)
  {
    double max_change = a_max_ * dt_;
    double diff = desired - current;
    
    if (std::abs(diff) > max_change) {
      return current + std::copysign(max_change, diff);
    }
    return desired;
  }
  
  double rateLimitYaw(double desired)
  {
    double max_change = yaw_accel_max_ * dt_;
    double diff = desired - last_cmd_.angular.z;
    
    double rate_limited;
    if (std::abs(diff) > max_change) {
      rate_limited = last_cmd_.angular.z + std::copysign(max_change, diff);
    } else {
      rate_limited = desired;
    }
    
    // Also clamp to max rate
    return clamp(rate_limited, -yaw_rate_max_, yaw_rate_max_);
  }
  
  geometry_msgs::msg::Twist smoothVelocity(const geometry_msgs::msg::Twist& desired)
  {
    geometry_msgs::msg::Twist smoothed;
    
    // Exponential moving average
    double alpha = velocity_smoothing_;
    smoothed.linear.x = alpha * last_cmd_.linear.x + (1.0 - alpha) * desired.linear.x;
    smoothed.linear.y = alpha * last_cmd_.linear.y + (1.0 - alpha) * desired.linear.y;
    smoothed.linear.z = alpha * last_cmd_.linear.z + (1.0 - alpha) * desired.linear.z;
    smoothed.angular.z = alpha * last_cmd_.angular.z + (1.0 - alpha) * desired.angular.z;
    
    return smoothed;
  }
  
  geometry_msgs::msg::Twist smoothStop()
  {
    geometry_msgs::msg::Twist cmd;
    
    // Gradually reduce velocity to zero
    double decay = 0.8;  // 80% of previous velocity
    cmd.linear.x = last_cmd_.linear.x * decay;
    cmd.linear.y = last_cmd_.linear.y * decay;
    cmd.linear.z = last_cmd_.linear.z * decay;
    cmd.angular.z = last_cmd_.angular.z * decay;
    
    // Zero out very small values
    if (std::abs(cmd.linear.x) < 0.01) cmd.linear.x = 0;
    if (std::abs(cmd.linear.y) < 0.01) cmd.linear.y = 0;
    if (std::abs(cmd.linear.z) < 0.01) cmd.linear.z = 0;
    if (std::abs(cmd.angular.z) < 0.01) cmd.angular.z = 0;
    
    // ═══════════════════════════════════════════════════════════════════
    // HEIGHT CONTROL: Even when stopping, maintain height
    // ═══════════════════════════════════════════════════════════════════
    if (have_odom_) {
      double current_z = current_pose_.position.z;
      double z_error = target_height_ - current_z;
      if (std::abs(z_error) > height_tolerance_) {
        cmd.linear.z = clamp(kp_z_ * z_error, -vz_max_, vz_max_);
      }
    }
    
    last_cmd_ = cmd;
    return cmd;
  }
  
  double clamp(double val, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, val));
  }
  
  // Parameters
  double control_rate_;
  double goal_tolerance_xy_, goal_tolerance_yaw_;
  double kp_linear_, kp_yaw_, kp_z_;
  double v_max_, yaw_rate_max_, vz_max_;
  double a_max_, yaw_accel_max_;
  double velocity_smoothing_;
  double slowdown_distance_;
  double min_approach_speed_;
  double progressive_yaw_distance_;
  double progressive_yaw_blend_;
  double target_height_;
  double height_tolerance_;
  double dt_;
  
  // State
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_twist_;
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Twist last_cmd_;
  bool have_odom_ = false;
  bool have_target_ = false;
  
  exploration_planner::msg::Trajectory::SharedPtr current_trajectory_;
  bool have_trajectory_ = false;
  
  // ROS
  rclcpp::Subscription<exploration_planner::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
