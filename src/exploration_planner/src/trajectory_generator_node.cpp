/**
 * Trajectory Generator Node
 *
 * Converts path to trajectory with smooth yaw injection.
 *
 * Input:  /exploration/planned_path (nav_msgs/Path)
 *         /odom - current state
 * Output: /exploration/trajectory (exploration_planner/Trajectory)
 *
 * Yaw Strategy:
 *   - Until last 1.5m: face movement direction
 *   - Last 1.5m: smoothly blend to target yaw
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "exploration_planner/msg/trajectory.hpp"
#include "exploration_planner/common.hpp"

using namespace exploration_planner;

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode() : Node("trajectory_generator")
  {
    declare_parameter("v_max", 2.5);
    declare_parameter("a_max", 0.7);
    declare_parameter("yaw_blend_distance", 1.5);

    v_max_ = get_parameter("v_max").as_double();
    a_max_ = get_parameter("a_max").as_double();
    yaw_blend_dist_ = get_parameter("yaw_blend_distance").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/exploration/planned_path", 10,
      std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));

    traj_pub_ = create_publisher<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10);

    // Path publisher for RViz visualization
    traj_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/exploration/trajectory_path", 10);

    RCLCPP_INFO(get_logger(), "Trajectory Generator: yaw_blend_dist=%.1fm", yaw_blend_dist_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    current_yaw_ = getYaw(current_pose_.orientation);
    have_odom_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!have_odom_ || msg->poses.empty()) return;

    auto traj = generateTrajectory(*msg);
    traj.header.stamp = now();
    traj.header.frame_id = msg->header.frame_id;
    traj_pub_->publish(traj);

    // Publish as Path for RViz visualization
    nav_msgs::msg::Path traj_path;
    traj_path.header = traj.header;
    traj_path.poses = traj.poses;
    traj_path_pub_->publish(traj_path);
  }

  exploration_planner::msg::Trajectory generateTrajectory(const nav_msgs::msg::Path& path)
  {
    exploration_planner::msg::Trajectory traj;
    if (path.poses.empty()) return traj;

    // Calculate cumulative distances from end (reverse)
    std::vector<double> dist_to_end(path.poses.size());
    dist_to_end.back() = 0.0;

    for (int i = path.poses.size() - 2; i >= 0; --i) {
      double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
      dist_to_end[i] = dist_to_end[i+1] + std::sqrt(dx*dx + dy*dy);
    }

    double total_length = dist_to_end[0];
    double target_yaw = getYaw(path.poses.back().pose.orientation);

    // Generate trajectory points
    double total_time = 0.0;
    geometry_msgs::msg::Point prev_pos = current_pose_.position;

    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto& wp = path.poses[i];

      // Distance from previous point
      double dx = wp.pose.position.x - prev_pos.x;
      double dy = wp.pose.position.y - prev_pos.y;
      double segment_dist = std::sqrt(dx*dx + dy*dy);
      double segment_time = computeTime(segment_dist);

      // Calculate yaw for this waypoint
      double waypoint_yaw;
      double d_to_end = dist_to_end[i];

      if (d_to_end > yaw_blend_dist_) {
        // Far from goal: face movement direction
        if (i < path.poses.size() - 1) {
          double next_dx = path.poses[i+1].pose.position.x - wp.pose.position.x;
          double next_dy = path.poses[i+1].pose.position.y - wp.pose.position.y;
          waypoint_yaw = std::atan2(next_dy, next_dx);
        } else {
          waypoint_yaw = target_yaw;
        }
      } else {
        // Last 1.5m: blend to target yaw
        double blend = 1.0 - (d_to_end / yaw_blend_dist_);
        blend = smoothstep(blend);

        // Movement direction
        double move_yaw;
        if (i < path.poses.size() - 1) {
          double next_dx = path.poses[i+1].pose.position.x - wp.pose.position.x;
          double next_dy = path.poses[i+1].pose.position.y - wp.pose.position.y;
          move_yaw = std::atan2(next_dy, next_dx);
        } else {
          move_yaw = target_yaw;
        }

        double yaw_diff = normalizeAngle(target_yaw - move_yaw);
        waypoint_yaw = normalizeAngle(move_yaw + blend * yaw_diff);
      }

      // Build pose
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = wp.pose.position;
      pose.pose.orientation = yawToQuaternion(waypoint_yaw);
      traj.poses.push_back(pose);

      // Velocity (direction towards next point)
      geometry_msgs::msg::Twist vel;
      if (segment_dist > 0.01 && segment_time > 0.01) {
        double speed = std::min(v_max_, segment_dist / segment_time);
        vel.linear.x = speed * dx / segment_dist;
        vel.linear.y = speed * dy / segment_dist;
      }
      traj.velocities.push_back(vel);

      total_time += segment_time;
      traj.time_from_start.push_back(total_time);

      prev_pos = wp.pose.position;
    }

    traj.total_time = total_time;
    return traj;
  }

  double computeTime(double dist)
  {
    if (dist < 0.01) return 0.1;
    double t_accel = v_max_ / a_max_;
    double d_accel = 0.5 * a_max_ * t_accel * t_accel;

    if (2 * d_accel >= dist) {
      return 2.0 * std::sqrt(dist / a_max_);
    }
    return 2.0 * t_accel + (dist - 2.0 * d_accel) / v_max_;
  }

  double smoothstep(double t)
  {
    t = std::max(0.0, std::min(1.0, t));
    return t * t * (3.0 - 2.0 * t);
  }

  double v_max_, a_max_, yaw_blend_dist_;
  geometry_msgs::msg::Pose current_pose_;
  double current_yaw_ = 0.0;
  bool have_odom_ = false;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<exploration_planner::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_path_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
