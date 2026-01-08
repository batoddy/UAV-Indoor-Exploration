/**
 * Node 4: Trajectory Follower
 * 
 * Input:  /exploration/trajectory (exploration_planner/Trajectory)
 *         /odom (current state)
 * Output: /cmd_vel (geometry_msgs/Twist)
 * 
 * Pure pursuit style trajectory following with velocity feedforward.
 * Outputs cmd_vel for the drone controller.
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
    declare_parameter("control_rate", 20.0);
    declare_parameter("lookahead_dist", 0.5);
    declare_parameter("goal_tolerance_xy", 0.2);
    declare_parameter("goal_tolerance_yaw", 0.1);
    
    // Gains
    declare_parameter("kp_xy", 1.0);
    declare_parameter("kp_yaw", 1.0);
    declare_parameter("kd_xy", 0.1);
    declare_parameter("kd_yaw", 0.1);
    
    // Limits
    declare_parameter("v_max", 1.0);
    declare_parameter("yaw_rate_max", 1.0);
    
    // Feedforward
    declare_parameter("use_feedforward", true);
    declare_parameter("ff_weight", 0.7);
    
    control_rate_ = get_parameter("control_rate").as_double();
    lookahead_dist_ = get_parameter("lookahead_dist").as_double();
    goal_tolerance_xy_ = get_parameter("goal_tolerance_xy").as_double();
    goal_tolerance_yaw_ = get_parameter("goal_tolerance_yaw").as_double();
    kp_xy_ = get_parameter("kp_xy").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    kd_xy_ = get_parameter("kd_xy").as_double();
    kd_yaw_ = get_parameter("kd_yaw").as_double();
    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    use_feedforward_ = get_parameter("use_feedforward").as_bool();
    ff_weight_ = get_parameter("ff_weight").as_double();
    
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
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&TrajectoryFollowerNode::controlLoop, this));
    
    RCLCPP_INFO(get_logger(), "Trajectory Follower initialized");
    RCLCPP_INFO(get_logger(), "  control_rate: %.1f Hz, lookahead: %.2f m", 
                control_rate_, lookahead_dist_);
    RCLCPP_INFO(get_logger(), "  kp_xy: %.2f, kp_yaw: %.2f", kp_xy_, kp_yaw_);
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
    trajectory_start_time_ = now();
    have_trajectory_ = true;
    
    RCLCPP_DEBUG(get_logger(), "Received trajectory with %zu points, duration: %.2f s",
                 msg->poses.size(), msg->total_time);
  }
  
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;
    
    if (!have_odom_ || !have_trajectory_ || !current_trajectory_ ||
        current_trajectory_->poses.empty()) {
      // Stop if no valid trajectory
      cmd_pub_->publish(cmd);
      return;
    }
    
    // Find current time along trajectory
    double t_elapsed = (now() - trajectory_start_time_).seconds();
    
    if (t_elapsed > current_trajectory_->total_time) {
      // Trajectory completed - hold position at last point
      t_elapsed = current_trajectory_->total_time;
    }
    
    // Find lookahead point on trajectory
    size_t target_idx = findLookaheadPoint(t_elapsed);
    
    if (target_idx >= current_trajectory_->poses.size()) {
      target_idx = current_trajectory_->poses.size() - 1;
    }
    
    const auto& target_pose = current_trajectory_->poses[target_idx];
    
    // Position error
    double ex = target_pose.pose.position.x - current_pose_.position.x;
    double ey = target_pose.pose.position.y - current_pose_.position.y;
    double dist_error = std::sqrt(ex*ex + ey*ey);
    
    // Yaw error
    double target_yaw = getYaw(target_pose.pose.orientation);
    double current_yaw = getYaw(current_pose_.orientation);
    double eyaw = normalizeAngle(target_yaw - current_yaw);
    
    // Check if at goal
    if (target_idx == current_trajectory_->poses.size() - 1 &&
        dist_error < goal_tolerance_xy_ && std::abs(eyaw) < goal_tolerance_yaw_) {
      // At goal - stop
      cmd_pub_->publish(cmd);
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "At goal");
      return;
    }
    
    // Feedforward from trajectory
    geometry_msgs::msg::Twist ff_vel;
    if (use_feedforward_ && target_idx < current_trajectory_->velocities.size()) {
      ff_vel = current_trajectory_->velocities[target_idx];
    }
    
    // PD control
    // Transform error to body frame
    double cos_yaw = std::cos(current_yaw);
    double sin_yaw = std::sin(current_yaw);
    double ex_body = cos_yaw * ex + sin_yaw * ey;
    double ey_body = -sin_yaw * ex + cos_yaw * ey;
    
    // Velocity error (in world frame, then transform)
    double evx = ff_vel.linear.x - current_twist_.linear.x;
    double evy = ff_vel.linear.y - current_twist_.linear.y;
    double evx_body = cos_yaw * evx + sin_yaw * evy;
    double evy_body = -sin_yaw * evx + cos_yaw * evy;
    
    // Compute control
    double vx_fb = kp_xy_ * ex_body + kd_xy_ * evx_body;
    double vy_fb = kp_xy_ * ey_body + kd_xy_ * evy_body;
    double vyaw_fb = kp_yaw_ * eyaw + kd_yaw_ * (ff_vel.angular.z - current_twist_.angular.z);
    
    // Feedforward + feedback
    if (use_feedforward_) {
      // Transform feedforward to body frame
      double ff_vx_body = cos_yaw * ff_vel.linear.x + sin_yaw * ff_vel.linear.y;
      double ff_vy_body = -sin_yaw * ff_vel.linear.x + cos_yaw * ff_vel.linear.y;
      
      cmd.linear.x = ff_weight_ * ff_vx_body + (1.0 - ff_weight_) * vx_fb;
      cmd.linear.y = ff_weight_ * ff_vy_body + (1.0 - ff_weight_) * vy_fb;
      cmd.angular.z = ff_weight_ * ff_vel.angular.z + (1.0 - ff_weight_) * vyaw_fb;
    } else {
      cmd.linear.x = vx_fb;
      cmd.linear.y = vy_fb;
      cmd.angular.z = vyaw_fb;
    }
    
    // Clamp velocities
    double v_linear = std::sqrt(cmd.linear.x*cmd.linear.x + cmd.linear.y*cmd.linear.y);
    if (v_linear > v_max_) {
      double scale = v_max_ / v_linear;
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
    }
    cmd.angular.z = std::max(-yaw_rate_max_, std::min(yaw_rate_max_, cmd.angular.z));
    
    cmd_pub_->publish(cmd);
  }
  
  size_t findLookaheadPoint(double t_elapsed)
  {
    if (!current_trajectory_ || current_trajectory_->poses.empty()) {
      return 0;
    }
    
    // Find point at t_elapsed + lookahead_time
    double lookahead_time = lookahead_dist_ / v_max_;  // Approximate
    double t_target = t_elapsed + lookahead_time;
    
    // Binary search in time_from_start
    const auto& times = current_trajectory_->time_from_start;
    
    size_t idx = 0;
    for (size_t i = 0; i < times.size(); ++i) {
      if (times[i] <= t_target) {
        idx = i;
      } else {
        break;
      }
    }
    
    return idx;
  }
  
  // Parameters
  double control_rate_;
  double lookahead_dist_;
  double goal_tolerance_xy_, goal_tolerance_yaw_;
  double kp_xy_, kp_yaw_, kd_xy_, kd_yaw_;
  double v_max_, yaw_rate_max_;
  bool use_feedforward_;
  double ff_weight_;
  
  // State
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_twist_;
  bool have_odom_ = false;
  
  exploration_planner::msg::Trajectory::SharedPtr current_trajectory_;
  rclcpp::Time trajectory_start_time_;
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