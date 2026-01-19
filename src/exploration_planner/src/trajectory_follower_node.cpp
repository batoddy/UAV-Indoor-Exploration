/**
 * Trajectory Follower Node
 *
 * Follows trajectory waypoints with proper yaw control.
 * Supports crab-walking (sideways flight) in the last 1.5m.
 *
 * Input:  /exploration/trajectory (exploration_planner/Trajectory)
 *         /odom - current state
 * Output: /cmd_vel (geometry_msgs/Twist)
 */

#include <rclcpp/rclcpp.hpp>
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
    declare_parameter("control_rate", 30.0);
    declare_parameter("goal_tolerance_xy", 0.25);
    declare_parameter("goal_tolerance_yaw", 0.15);
    declare_parameter("waypoint_tolerance", 0.5);

    declare_parameter("kp_linear", 1.0);
    declare_parameter("kp_yaw", 1.5);
    declare_parameter("kp_z", 1.0);

    declare_parameter("v_max", 2.5);
    declare_parameter("yaw_rate_max", 1.5);
    declare_parameter("vz_max", 1.0);

    declare_parameter("a_max", 0.8);
    declare_parameter("yaw_accel_max", 1.0);
    declare_parameter("velocity_smoothing", 0.2);

    declare_parameter("slowdown_distance", 1.5);
    declare_parameter("min_approach_speed", 0.3);

    declare_parameter("target_height", 1.5);
    declare_parameter("height_tolerance", 0.2);

    loadParams();

    traj_sub_ = create_subscription<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10,
      std::bind(&TrajectoryFollowerNode::trajCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryFollowerNode::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&TrajectoryFollowerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Trajectory Follower initialized");
  }

private:
  void loadParams()
  {
    dt_ = 1.0 / get_parameter("control_rate").as_double();
    goal_tol_xy_ = get_parameter("goal_tolerance_xy").as_double();
    goal_tol_yaw_ = get_parameter("goal_tolerance_yaw").as_double();
    wp_tol_ = get_parameter("waypoint_tolerance").as_double();

    kp_lin_ = get_parameter("kp_linear").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    kp_z_ = get_parameter("kp_z").as_double();

    v_max_ = get_parameter("v_max").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
    vz_max_ = get_parameter("vz_max").as_double();

    a_max_ = get_parameter("a_max").as_double();
    yaw_accel_max_ = get_parameter("yaw_accel_max").as_double();
    smoothing_ = get_parameter("velocity_smoothing").as_double();

    slowdown_dist_ = get_parameter("slowdown_distance").as_double();
    min_speed_ = get_parameter("min_approach_speed").as_double();

    target_z_ = get_parameter("target_height").as_double();
    z_tol_ = get_parameter("height_tolerance").as_double();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos_ = msg->pose.pose.position;
    yaw_ = getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void trajCallback(const exploration_planner::msg::Trajectory::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    traj_ = *msg;
    current_wp_ = 0;
    have_traj_ = true;

    RCLCPP_INFO(get_logger(), "New trajectory: %zu waypoints", traj_.poses.size());
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!have_odom_ || !have_traj_ || traj_.poses.empty()) {
      cmd = smoothStop();
      cmd_pub_->publish(cmd);
      return;
    }

    // Find current target waypoint
    updateCurrentWaypoint();

    if (current_wp_ >= traj_.poses.size()) {
      // Trajectory complete
      cmd = smoothStop();
      cmd_pub_->publish(cmd);
      return;
    }

    // Get target pose (current waypoint)
    const auto& target = traj_.poses[current_wp_].pose;
    double target_yaw = getYaw(target.orientation);

    // Final goal (last waypoint)
    const auto& goal = traj_.poses.back().pose;
    double goal_yaw = getYaw(goal.orientation);

    // Errors
    double dx = target.position.x - pos_.x;
    double dy = target.position.y - pos_.y;
    double dist_to_wp = std::sqrt(dx*dx + dy*dy);

    double dx_goal = goal.position.x - pos_.x;
    double dy_goal = goal.position.y - pos_.y;
    double dist_to_goal = std::sqrt(dx_goal*dx_goal + dy_goal*dy_goal);

    // Height control
    double vz = 0.0;
    double z_err = target_z_ - pos_.z;
    if (std::abs(z_err) > z_tol_) {
      vz = clamp(kp_z_ * z_err, -vz_max_, vz_max_);
    }

    // At final goal?
    if (dist_to_goal < goal_tol_xy_) {
      double yaw_err = normalizeAngle(goal_yaw - yaw_);
      if (std::abs(yaw_err) < goal_tol_yaw_) {
        cmd = smoothStop();
        cmd.linear.z = vz;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "At goal!");
        return;
      }
      // Rotate to final yaw
      cmd.angular.z = rateLimit(kp_yaw_ * yaw_err, last_cmd_.angular.z, yaw_accel_max_);
      cmd.angular.z = clamp(cmd.angular.z, -yaw_rate_max_, yaw_rate_max_);
      cmd.linear.z = vz;
      cmd = smooth(cmd);
      cmd_pub_->publish(cmd);
      return;
    }

    // ═══════════════════════════════════════════════════════════════════
    // YAW CONTROL: Follow trajectory yaw (allows crab-walking)
    // ═══════════════════════════════════════════════════════════════════
    double desired_yaw = target_yaw;
    double yaw_err = normalizeAngle(desired_yaw - yaw_);

    // ═══════════════════════════════════════════════════════════════════
    // VELOCITY: Move towards target in WORLD frame, then transform to body
    // ═══════════════════════════════════════════════════════════════════

    // Speed scaling
    double speed_scale = 1.0;
    if (dist_to_goal < slowdown_dist_) {
      speed_scale = std::max(min_speed_ / v_max_, dist_to_goal / slowdown_dist_);
    }

    // World frame velocity (towards waypoint)
    double vx_world = kp_lin_ * dx * speed_scale;
    double vy_world = kp_lin_ * dy * speed_scale;

    // Clamp world velocity
    double v_world = std::sqrt(vx_world*vx_world + vy_world*vy_world);
    if (v_world > v_max_ * speed_scale) {
      double scale = v_max_ * speed_scale / v_world;
      vx_world *= scale;
      vy_world *= scale;
    }

    // Transform to body frame (considering current yaw)
    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);
    double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
    double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

    // Rate limiting
    cmd.linear.x = rateLimit(vx_body, last_cmd_.linear.x, a_max_);
    cmd.linear.y = rateLimit(vy_body, last_cmd_.linear.y, a_max_);
    cmd.linear.z = rateLimit(vz, last_cmd_.linear.z, a_max_);
    cmd.angular.z = rateLimit(kp_yaw_ * yaw_err, last_cmd_.angular.z, yaw_accel_max_);
    cmd.angular.z = clamp(cmd.angular.z, -yaw_rate_max_, yaw_rate_max_);

    // Smoothing
    cmd = smooth(cmd);

    cmd_pub_->publish(cmd);
    last_cmd_ = cmd;
  }

  void updateCurrentWaypoint()
  {
    if (traj_.poses.empty()) return;

    // Check if we reached current waypoint
    while (current_wp_ < traj_.poses.size() - 1) {
      const auto& wp = traj_.poses[current_wp_].pose;
      double dx = wp.position.x - pos_.x;
      double dy = wp.position.y - pos_.y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist < wp_tol_) {
        current_wp_++;
        RCLCPP_DEBUG(get_logger(), "Reached waypoint %zu", current_wp_);
      } else {
        break;
      }
    }
  }

  double rateLimit(double desired, double current, double max_rate)
  {
    double max_change = max_rate * dt_;
    double diff = desired - current;
    if (std::abs(diff) > max_change) {
      return current + std::copysign(max_change, diff);
    }
    return desired;
  }

  geometry_msgs::msg::Twist smooth(const geometry_msgs::msg::Twist& cmd)
  {
    geometry_msgs::msg::Twist out;
    double a = smoothing_;
    out.linear.x = a * last_cmd_.linear.x + (1-a) * cmd.linear.x;
    out.linear.y = a * last_cmd_.linear.y + (1-a) * cmd.linear.y;
    out.linear.z = a * last_cmd_.linear.z + (1-a) * cmd.linear.z;
    out.angular.z = a * last_cmd_.angular.z + (1-a) * cmd.angular.z;
    return out;
  }

  geometry_msgs::msg::Twist smoothStop()
  {
    geometry_msgs::msg::Twist cmd;
    double decay = 0.85;
    cmd.linear.x = last_cmd_.linear.x * decay;
    cmd.linear.y = last_cmd_.linear.y * decay;
    cmd.linear.z = last_cmd_.linear.z * decay;
    cmd.angular.z = last_cmd_.angular.z * decay;

    if (std::abs(cmd.linear.x) < 0.01) cmd.linear.x = 0;
    if (std::abs(cmd.linear.y) < 0.01) cmd.linear.y = 0;
    if (std::abs(cmd.linear.z) < 0.01) cmd.linear.z = 0;
    if (std::abs(cmd.angular.z) < 0.01) cmd.angular.z = 0;

    // Height maintenance
    if (have_odom_) {
      double z_err = target_z_ - pos_.z;
      if (std::abs(z_err) > z_tol_) {
        cmd.linear.z = clamp(kp_z_ * z_err, -vz_max_, vz_max_);
      }
    }

    last_cmd_ = cmd;
    return cmd;
  }

  double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

  // Parameters
  double dt_, goal_tol_xy_, goal_tol_yaw_, wp_tol_;
  double kp_lin_, kp_yaw_, kp_z_;
  double v_max_, yaw_rate_max_, vz_max_;
  double a_max_, yaw_accel_max_, smoothing_;
  double slowdown_dist_, min_speed_;
  double target_z_, z_tol_;

  // State
  geometry_msgs::msg::Point pos_;
  double yaw_ = 0.0;
  bool have_odom_ = false;
  bool have_traj_ = false;

  exploration_planner::msg::Trajectory traj_;
  size_t current_wp_ = 0;
  geometry_msgs::msg::Twist last_cmd_;

  // ROS
  rclcpp::Subscription<exploration_planner::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
