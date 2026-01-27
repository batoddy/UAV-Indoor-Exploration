/**
 * Trajectory Follower Node - Pure Pursuit Algorithm
 *
 * Follows trajectory using Pure Pursuit path following.
 * Instead of going directly to waypoints, follows the path precisely.
 *
 * Pure Pursuit Algorithm:
 *   1. Find closest point on path
 *   2. Find lookahead point (lookahead_distance ahead on path)
 *   3. Steer towards lookahead point
 *   4. This ensures UAV stays on path, no shortcuts
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

#include <limits>
#include <tuple>
#include <vector>

using namespace exploration_planner;

class TrajectoryFollowerNode : public rclcpp::Node
{
public:
  TrajectoryFollowerNode() : Node("trajectory_follower")
  {
    declare_parameter("control_rate", 30.0);
    declare_parameter("goal_tolerance_xy", 0.25);
    declare_parameter("goal_tolerance_yaw", 0.15);

    declare_parameter("kp_linear", 1.0);
    declare_parameter("kd_linear", 0.3);
    declare_parameter("kp_yaw", 1.5);
    declare_parameter("kd_yaw", 0.2);
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

    // Pure Pursuit parameters
    declare_parameter("lookahead_distance", 1.0);      // [m] Lookahead distance
    declare_parameter("min_lookahead_distance", 0.5);  // [m] Minimum lookahead
    declare_parameter("lookahead_gain", 0.5);          // Lookahead scales with velocity

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

    kp_lin_ = get_parameter("kp_linear").as_double();
    kd_lin_ = get_parameter("kd_linear").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    kd_yaw_ = get_parameter("kd_yaw").as_double();
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

    // Pure Pursuit
    lookahead_dist_ = get_parameter("lookahead_distance").as_double();
    min_lookahead_ = get_parameter("min_lookahead_distance").as_double();
    lookahead_gain_ = get_parameter("lookahead_gain").as_double();
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
    closest_idx_ = 0;
    have_traj_ = true;

    // Precompute cumulative distances along path
    path_distances_.clear();
    path_distances_.push_back(0.0);
    for (size_t i = 1; i < traj_.poses.size(); ++i) {
      double dx = traj_.poses[i].pose.position.x - traj_.poses[i-1].pose.position.x;
      double dy = traj_.poses[i].pose.position.y - traj_.poses[i-1].pose.position.y;
      path_distances_.push_back(path_distances_.back() + std::sqrt(dx*dx + dy*dy));
    }
    total_path_length_ = path_distances_.back();

    // Reset derivative state for new trajectory
    prev_yaw_err_ = 0.0;
    prev_dx_ = 0.0;
    prev_dy_ = 0.0;

    RCLCPP_INFO(get_logger(), "New trajectory: %zu waypoints, length: %.2fm",
                traj_.poses.size(), total_path_length_);
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!have_odom_ || !have_traj_ || traj_.poses.empty()) {
      cmd = smoothStop();
      cmd_pub_->publish(cmd);
      return;
    }

    // ═══════════════════════════════════════════════════════════════════
    // PURE PURSUIT: Find closest point and lookahead point on path
    // ═══════════════════════════════════════════════════════════════════

    // Step 1: Find closest point on path (only search forward to avoid going back)
    updateClosestPoint();

    // Step 2: Calculate dynamic lookahead distance based on current velocity
    double current_speed = std::sqrt(last_cmd_.linear.x * last_cmd_.linear.x +
                                      last_cmd_.linear.y * last_cmd_.linear.y);
    double lookahead = std::max(min_lookahead_, lookahead_dist_ + lookahead_gain_ * current_speed);

    // Step 3: Find lookahead point on path
    auto [lookahead_pos, lookahead_yaw, dist_to_end] = findLookaheadPoint(lookahead);

    // Final goal
    const auto& goal = traj_.poses.back().pose;
    double goal_yaw = getYaw(goal.orientation);

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
        have_traj_ = false;  // Trajectory complete
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "At goal!");
        return;
      }
      // Rotate to final yaw
      cmd.angular.z = rateLimit(kp_yaw_ * yaw_err, last_cmd_.angular.z, yaw_accel_max_);
      cmd.angular.z = clamp(cmd.angular.z, -yaw_rate_max_, yaw_rate_max_);
      cmd.linear.z = vz;
      cmd = smooth(cmd);
      cmd_pub_->publish(cmd);
      last_cmd_ = cmd;
      return;
    }

    // ═══════════════════════════════════════════════════════════════════
    // YAW CONTROL: Follow path yaw from trajectory
    // ═══════════════════════════════════════════════════════════════════
    double desired_yaw = lookahead_yaw;
    double yaw_err = normalizeAngle(desired_yaw - yaw_);

    // Derivative term for yaw
    double yaw_err_derivative = (yaw_err - prev_yaw_err_) / dt_;
    prev_yaw_err_ = yaw_err;

    // PD yaw control
    double yaw_cmd = kp_yaw_ * yaw_err + kd_yaw_ * yaw_err_derivative;

    // ═══════════════════════════════════════════════════════════════════
    // PURE PURSUIT VELOCITY: Steer towards lookahead point
    // ═══════════════════════════════════════════════════════════════════

    // Error to lookahead point
    double dx = lookahead_pos.x - pos_.x;
    double dy = lookahead_pos.y - pos_.y;
    double dist_to_lookahead = std::sqrt(dx*dx + dy*dy);

    // Speed scaling near goal
    double speed_scale = 1.0;
    if (dist_to_end < slowdown_dist_) {
      speed_scale = std::max(min_speed_ / v_max_, dist_to_end / slowdown_dist_);
    }

    // Derivative terms for position
    double dx_derivative = (dx - prev_dx_) / dt_;
    double dy_derivative = (dy - prev_dy_) / dt_;
    prev_dx_ = dx;
    prev_dy_ = dy;

    // PD control for world frame velocity
    double vx_world = (kp_lin_ * dx + kd_lin_ * dx_derivative) * speed_scale;
    double vy_world = (kp_lin_ * dy + kd_lin_ * dy_derivative) * speed_scale;

    // Clamp world velocity
    double v_world = std::sqrt(vx_world*vx_world + vy_world*vy_world);
    if (v_world > v_max_ * speed_scale) {
      double scale = v_max_ * speed_scale / v_world;
      vx_world *= scale;
      vy_world *= scale;
    }

    // Transform to body frame
    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);
    double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
    double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

    // Rate limiting
    cmd.linear.x = rateLimit(vx_body, last_cmd_.linear.x, a_max_);
    cmd.linear.y = rateLimit(vy_body, last_cmd_.linear.y, a_max_);
    cmd.linear.z = rateLimit(vz, last_cmd_.linear.z, a_max_);
    cmd.angular.z = rateLimit(yaw_cmd, last_cmd_.angular.z, yaw_accel_max_);
    cmd.angular.z = clamp(cmd.angular.z, -yaw_rate_max_, yaw_rate_max_);

    // Smoothing
    cmd = smooth(cmd);

    cmd_pub_->publish(cmd);
    last_cmd_ = cmd;
  }

  // Find closest point on path (only searches forward)
  void updateClosestPoint()
  {
    if (traj_.poses.empty()) return;

    double min_dist = std::numeric_limits<double>::max();
    size_t best_idx = closest_idx_;

    // Search from current closest to end (don't go backwards)
    for (size_t i = closest_idx_; i < traj_.poses.size(); ++i) {
      double dx = traj_.poses[i].pose.position.x - pos_.x;
      double dy = traj_.poses[i].pose.position.y - pos_.y;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist < min_dist) {
        min_dist = dist;
        best_idx = i;
      }

      // Stop searching if distance starts increasing significantly
      // (we've passed the closest point)
      if (dist > min_dist + 0.5 && i > best_idx + 3) {
        break;
      }
    }

    closest_idx_ = best_idx;
  }

  // Find lookahead point on path, returns (position, yaw, distance_to_end)
  std::tuple<geometry_msgs::msg::Point, double, double> findLookaheadPoint(double lookahead_dist)
  {
    if (traj_.poses.empty() || closest_idx_ >= traj_.poses.size()) {
      return {pos_, yaw_, 0.0};
    }

    // Distance along path at closest point
    double closest_path_dist = path_distances_[closest_idx_];

    // Target distance along path
    double target_dist = closest_path_dist + lookahead_dist;

    // Distance to end
    double dist_to_end = total_path_length_ - closest_path_dist;

    // Find the segment containing the target distance
    size_t lookahead_idx = closest_idx_;
    for (size_t i = closest_idx_; i < traj_.poses.size(); ++i) {
      if (path_distances_[i] >= target_dist) {
        lookahead_idx = i;
        break;
      }
      lookahead_idx = i;
    }

    // If we're at the last point, return it
    if (lookahead_idx >= traj_.poses.size() - 1) {
      const auto& last_pose = traj_.poses.back().pose;
      return {last_pose.position, getYaw(last_pose.orientation), dist_to_end};
    }

    // Interpolate between waypoints for smooth lookahead
    double seg_start_dist = path_distances_[lookahead_idx];
    double seg_end_dist = path_distances_[lookahead_idx + 1];
    double seg_length = seg_end_dist - seg_start_dist;

    double t = 0.0;
    if (seg_length > 0.001) {
      t = (target_dist - seg_start_dist) / seg_length;
      t = clamp(t, 0.0, 1.0);
    }

    const auto& p1 = traj_.poses[lookahead_idx].pose;
    const auto& p2 = traj_.poses[lookahead_idx + 1].pose;

    geometry_msgs::msg::Point interp_pos;
    interp_pos.x = p1.position.x + t * (p2.position.x - p1.position.x);
    interp_pos.y = p1.position.y + t * (p2.position.y - p1.position.y);
    interp_pos.z = p1.position.z + t * (p2.position.z - p1.position.z);

    // Interpolate yaw (handling wrap-around)
    double yaw1 = getYaw(p1.orientation);
    double yaw2 = getYaw(p2.orientation);
    double yaw_diff = normalizeAngle(yaw2 - yaw1);
    double interp_yaw = normalizeAngle(yaw1 + t * yaw_diff);

    return {interp_pos, interp_yaw, dist_to_end};
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
  double dt_, goal_tol_xy_, goal_tol_yaw_;
  double kp_lin_, kd_lin_, kp_yaw_, kd_yaw_, kp_z_;
  double v_max_, yaw_rate_max_, vz_max_;
  double a_max_, yaw_accel_max_, smoothing_;
  double slowdown_dist_, min_speed_;
  double target_z_, z_tol_;

  // Pure Pursuit parameters
  double lookahead_dist_, min_lookahead_, lookahead_gain_;

  // State
  geometry_msgs::msg::Point pos_;
  double yaw_ = 0.0;
  bool have_odom_ = false;
  bool have_traj_ = false;

  // Trajectory and Pure Pursuit state
  exploration_planner::msg::Trajectory traj_;
  size_t closest_idx_ = 0;                    // Index of closest point on path
  std::vector<double> path_distances_;        // Cumulative distances along path
  double total_path_length_ = 0.0;
  geometry_msgs::msg::Twist last_cmd_;

  // PD control - previous errors for derivative
  double prev_yaw_err_ = 0.0;
  double prev_dx_ = 0.0;
  double prev_dy_ = 0.0;

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
