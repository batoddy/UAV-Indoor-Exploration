/**
 * Path Yaw Injector Node
 *
 * Injects smooth yaw transitions into Nav2 paths.
 *
 * Input:  /exploration/nav2_path (nav_msgs/Path) - raw path from Nav2
 *         /odom - current pose for initial yaw
 * Output: /exploration/planned_path (nav_msgs/Path) - path with yaw injected
 *
 * Yaw Distribution:
 *   - First X meters: current_yaw → movement_direction
 *   - Middle: movement_direction (face path direction)
 *   - Last Y meters: movement_direction → target_yaw (face frontier)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "exploration_planner/common.hpp"

using namespace exploration_planner;

class PathYawInjectorNode : public rclcpp::Node
{
public:
  PathYawInjectorNode() : Node("path_yaw_injector")
  {
    declare_parameter("yaw_initial_blend_dist", 1.0);
    declare_parameter("yaw_final_blend_dist", 2.5);

    initial_blend_dist_ = get_parameter("yaw_initial_blend_dist").as_double();
    final_blend_dist_ = get_parameter("yaw_final_blend_dist").as_double();

    // Subscribers
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/exploration/nav2_path", 10,
      std::bind(&PathYawInjectorNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PathYawInjectorNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/planned_path", 10);

    RCLCPP_INFO(get_logger(), "Path Yaw Injector initialized");
    RCLCPP_INFO(get_logger(), "  Blend distances: initial=%.1fm, final=%.1fm",
                initial_blend_dist_, final_blend_dist_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_yaw_ = getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odom received");
      return;
    }

    if (msg->poses.size() < 2) {
      path_pub_->publish(*msg);
      return;
    }

    auto injected_path = injectYaw(*msg);
    path_pub_->publish(injected_path);

    RCLCPP_DEBUG(get_logger(), "Injected yaw into path with %zu poses", injected_path.poses.size());
  }

  nav_msgs::msg::Path injectYaw(const nav_msgs::msg::Path& path)
  {
    nav_msgs::msg::Path result;
    result.header = path.header;

    if (path.poses.empty()) return result;

    // Calculate cumulative distances
    std::vector<double> cumulative_dist;
    cumulative_dist.push_back(0.0);
    double total_length = 0.0;

    for (size_t i = 1; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
      total_length += std::sqrt(dx*dx + dy*dy);
      cumulative_dist.push_back(total_length);
    }

    // Adaptive blend distances for short paths
    double initial_blend = initial_blend_dist_;
    double final_blend = final_blend_dist_;

    if (total_length < initial_blend + final_blend) {
      if (total_length < 0.5) {
        initial_blend = 0.0;
        final_blend = total_length;
      } else {
        double scale = total_length / (initial_blend + final_blend);
        initial_blend *= scale * 0.3;
        final_blend *= scale * 0.7;
      }
    }

    // Get yaw values
    double initial_yaw = current_yaw_;
    double target_yaw = getYaw(path.poses.back().pose.orientation);

    // Calculate overall movement direction
    double dx = path.poses.back().pose.position.x - path.poses.front().pose.position.x;
    double dy = path.poses.back().pose.position.y - path.poses.front().pose.position.y;
    double movement_yaw = std::atan2(dy, dx);

    // Process each pose
    for (size_t i = 0; i < path.poses.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose = path.poses[i];

      double dist_from_start = cumulative_dist[i];
      double dist_to_end = total_length - dist_from_start;

      double waypoint_yaw = computeYaw(
        dist_from_start, dist_to_end, total_length,
        initial_blend, final_blend,
        initial_yaw, movement_yaw, target_yaw,
        path, i);

      pose.pose.orientation = yawToQuaternion(waypoint_yaw);
      result.poses.push_back(pose);
    }

    return result;
  }

  double computeYaw(double dist_from_start, double dist_to_end, double total_length,
                    double initial_blend, double final_blend,
                    double initial_yaw, double movement_yaw, double target_yaw,
                    const nav_msgs::msg::Path& path, size_t idx)
  {
    bool in_initial = (dist_from_start < initial_blend) && (initial_blend > 0.01);
    bool in_final = (dist_to_end < final_blend) && (final_blend > 0.01);

    if (in_initial && in_final) {
      // Overlapping zones - blend directly from initial to target
      double blend = smoothstep(dist_from_start / total_length);
      double yaw_diff = normalizeAngle(target_yaw - initial_yaw);
      return normalizeAngle(initial_yaw + blend * yaw_diff);
    }

    if (in_initial) {
      // Blend from current_yaw to movement_yaw
      double blend = smoothstep(dist_from_start / initial_blend);
      double yaw_diff = normalizeAngle(movement_yaw - initial_yaw);
      return normalizeAngle(initial_yaw + blend * yaw_diff);
    }

    if (in_final) {
      // Blend from movement_yaw to target_yaw
      double blend = smoothstep(1.0 - (dist_to_end / final_blend));
      double yaw_diff = normalizeAngle(target_yaw - movement_yaw);
      return normalizeAngle(movement_yaw + blend * yaw_diff);
    }

    // Middle - face movement direction
    if (idx < path.poses.size() - 1) {
      double next_dx = path.poses[idx+1].pose.position.x - path.poses[idx].pose.position.x;
      double next_dy = path.poses[idx+1].pose.position.y - path.poses[idx].pose.position.y;
      if (std::sqrt(next_dx*next_dx + next_dy*next_dy) > 0.01) {
        return std::atan2(next_dy, next_dx);
      }
    }

    return movement_yaw;
  }

  double smoothstep(double t)
  {
    t = std::max(0.0, std::min(1.0, t));
    return t * t * (3.0 - 2.0 * t);
  }

  // Parameters
  double initial_blend_dist_;
  double final_blend_dist_;

  // State
  double current_yaw_ = 0.0;
  bool have_odom_ = false;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathYawInjectorNode>());
  rclcpp::shutdown();
  return 0;
}
