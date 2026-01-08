/**
 * Local Path Adjuster Node
 * 
 * Input:  /exploration/planned_path (nav_msgs/Path) - from global path planner
 *         /camera/depth/points (sensor_msgs/PointCloud2) - from depth camera
 *         /odom (nav_msgs/Odometry) - drone position
 * Output: /exploration/adjusted_path (nav_msgs/Path) - obstacle-free path
 * 
 * Checks for obstacles in the flight corridor (target_height ± height_tolerance)
 * and adjusts the path to avoid them using local pointcloud data.
 * 
 * This provides real-time obstacle avoidance that the global planner might miss.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

class LocalPathAdjusterNode : public rclcpp::Node
{
public:
  LocalPathAdjusterNode() : Node("local_path_adjuster")
  {
    // Enable/disable parameter
    declare_parameter("local_path_adjuster_enabled", true);
    enabled_ = get_parameter("local_path_adjuster_enabled").as_bool();
    
    // Height parameters
    declare_parameter("target_height", 1.5);
    declare_parameter("height_check_tolerance", 0.2);  // ±20cm around target height
    
    // Safety parameters
    declare_parameter("robot_radius", 0.5);            // Robot collision radius
    declare_parameter("safety_margin", 0.3);           // Extra safety margin
    declare_parameter("check_distance", 3.0);          // How far ahead to check
    declare_parameter("adjustment_step", 0.2);         // Path adjustment step size
    declare_parameter("max_adjustment", 1.0);          // Maximum lateral adjustment
    
    // Pointcloud parameters
    declare_parameter("pointcloud_topic", "/camera/depth/points");
    declare_parameter("min_points_for_obstacle", 5);   // Minimum points to consider as obstacle
    declare_parameter("voxel_size", 0.1);              // Voxel grid size for downsampling
    
    // Frame parameters
    declare_parameter("world_frame", "map");
    
    target_height_ = get_parameter("target_height").as_double();
    height_tolerance_ = get_parameter("height_check_tolerance").as_double();
    robot_radius_ = get_parameter("robot_radius").as_double();
    safety_margin_ = get_parameter("safety_margin").as_double();
    check_distance_ = get_parameter("check_distance").as_double();
    adjustment_step_ = get_parameter("adjustment_step").as_double();
    max_adjustment_ = get_parameter("max_adjustment").as_double();
    std::string pointcloud_topic = get_parameter("pointcloud_topic").as_string();
    min_points_for_obstacle_ = get_parameter("min_points_for_obstacle").as_int();
    voxel_size_ = get_parameter("voxel_size").as_double();
    world_frame_ = get_parameter("world_frame").as_string();
    
    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribers
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/exploration/planned_path", 10,
      std::bind(&LocalPathAdjusterNode::pathCallback, this, std::placeholders::_1));
    
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&LocalPathAdjusterNode::cloudCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&LocalPathAdjusterNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    adjusted_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/exploration/adjusted_path", 10);
    
    if (enabled_) {
      RCLCPP_INFO(get_logger(), "Local Path Adjuster initialized [ENABLED]");
      RCLCPP_INFO(get_logger(), "  Height check: %.2f ± %.2fm", target_height_, height_tolerance_);
      RCLCPP_INFO(get_logger(), "  Robot radius: %.2fm, safety margin: %.2fm", robot_radius_, safety_margin_);
      RCLCPP_INFO(get_logger(), "  Check distance: %.2fm", check_distance_);
    } else {
      RCLCPP_INFO(get_logger(), "Local Path Adjuster initialized [DISABLED] - passing through paths");
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    have_odom_ = true;
  }
  
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Transform pointcloud to world frame and extract obstacle points
    try {
      // Get transform
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
        world_frame_, msg->header.frame_id, 
        msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      
      // Clear previous obstacles
      obstacle_points_.clear();
      
      // Height filter bounds
      double z_min = target_height_ - height_tolerance_;
      double z_max = target_height_ + height_tolerance_;
      
      // Iterate through pointcloud
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // Skip invalid points
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }
        
        // Transform point to world frame
        geometry_msgs::msg::Point pt_camera;
        pt_camera.x = *iter_x;
        pt_camera.y = *iter_y;
        pt_camera.z = *iter_z;
        
        geometry_msgs::msg::Point pt_world = transformPoint(pt_camera, transform);
        
        // Height filter - only keep points in flight corridor
        if (pt_world.z >= z_min && pt_world.z <= z_max) {
          obstacle_points_.push_back(pt_world);
        }
      }
      
      have_cloud_ = true;
      
      RCLCPP_DEBUG(get_logger(), "Obstacle points in corridor: %zu", obstacle_points_.size());
      
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                           "TF transform failed: %s", ex.what());
    }
  }
  
  geometry_msgs::msg::Point transformPoint(
    const geometry_msgs::msg::Point& pt_in,
    const geometry_msgs::msg::TransformStamped& transform)
  {
    geometry_msgs::msg::Point pt_out;
    
    // Apply rotation
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;
    
    // Quaternion rotation
    double t2 = qw * pt_in.x + qy * pt_in.z - qz * pt_in.y;
    double t3 = qw * pt_in.y + qz * pt_in.x - qx * pt_in.z;
    double t4 = qw * pt_in.z + qx * pt_in.y - qy * pt_in.x;
    double t5 = -qx * pt_in.x - qy * pt_in.y - qz * pt_in.z;
    
    pt_out.x = t2 * qw + t5 * (-qx) + t3 * (-qz) - t4 * (-qy);
    pt_out.y = t3 * qw + t5 * (-qy) + t4 * (-qx) - t2 * (-qz);
    pt_out.z = t4 * qw + t5 * (-qz) + t2 * (-qy) - t3 * (-qx);
    
    // Apply translation
    pt_out.x += transform.transform.translation.x;
    pt_out.y += transform.transform.translation.y;
    pt_out.z += transform.transform.translation.z;
    
    return pt_out;
  }
  
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // If disabled, just pass through the original path
    if (!enabled_) {
      adjusted_path_pub_->publish(*msg);
      return;
    }
    
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odometry received yet");
      adjusted_path_pub_->publish(*msg);  // Pass through original
      return;
    }
    
    if (msg->poses.empty()) {
      adjusted_path_pub_->publish(*msg);
      return;
    }
    
    if (!have_cloud_ || obstacle_points_.empty()) {
      // No obstacle data - pass through original path
      adjusted_path_pub_->publish(*msg);
      return;
    }
    
    // Adjust path based on obstacles
    nav_msgs::msg::Path adjusted_path = adjustPath(*msg);
    
    adjusted_path_pub_->publish(adjusted_path);
  }
  
  nav_msgs::msg::Path adjustPath(const nav_msgs::msg::Path& original_path)
  {
    nav_msgs::msg::Path adjusted_path;
    adjusted_path.header = original_path.header;
    
    double collision_radius = robot_radius_ + safety_margin_;
    
    for (size_t i = 0; i < original_path.poses.size(); ++i) {
      const auto& pose = original_path.poses[i];
      
      // Check distance from drone
      double dist_from_drone = std::hypot(
        pose.pose.position.x - current_pose_.position.x,
        pose.pose.position.y - current_pose_.position.y);
      
      // Only check points within check_distance
      if (dist_from_drone > check_distance_) {
        adjusted_path.poses.push_back(pose);
        continue;
      }
      
      // Check for obstacles near this waypoint
      bool has_obstacle = checkObstacleNearPoint(
        pose.pose.position.x, 
        pose.pose.position.y, 
        collision_radius);
      
      if (!has_obstacle) {
        // No obstacle - keep original
        adjusted_path.poses.push_back(pose);
      } else {
        // Obstacle detected - try to adjust
        geometry_msgs::msg::PoseStamped adjusted_pose = adjustWaypoint(pose, original_path, i);
        adjusted_path.poses.push_back(adjusted_pose);
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Adjusted waypoint %zu: (%.2f,%.2f) -> (%.2f,%.2f)",
                             i, pose.pose.position.x, pose.pose.position.y,
                             adjusted_pose.pose.position.x, adjusted_pose.pose.position.y);
      }
    }
    
    return adjusted_path;
  }
  
  bool checkObstacleNearPoint(double x, double y, double radius)
  {
    int obstacle_count = 0;
    
    for (const auto& pt : obstacle_points_) {
      double dist = std::hypot(pt.x - x, pt.y - y);
      if (dist < radius) {
        obstacle_count++;
        if (obstacle_count >= min_points_for_obstacle_) {
          return true;
        }
      }
    }
    
    return false;
  }
  
  geometry_msgs::msg::PoseStamped adjustWaypoint(
    const geometry_msgs::msg::PoseStamped& original,
    const nav_msgs::msg::Path& path,
    size_t waypoint_idx)
  {
    geometry_msgs::msg::PoseStamped adjusted = original;
    
    // Calculate path direction for perpendicular adjustment
    double path_dx = 0.0, path_dy = 0.0;
    
    if (waypoint_idx < path.poses.size() - 1) {
      path_dx = path.poses[waypoint_idx + 1].pose.position.x - original.pose.position.x;
      path_dy = path.poses[waypoint_idx + 1].pose.position.y - original.pose.position.y;
    } else if (waypoint_idx > 0) {
      path_dx = original.pose.position.x - path.poses[waypoint_idx - 1].pose.position.x;
      path_dy = original.pose.position.y - path.poses[waypoint_idx - 1].pose.position.y;
    }
    
    double path_len = std::hypot(path_dx, path_dy);
    if (path_len < 0.01) {
      return adjusted;  // Can't determine direction
    }
    
    // Normalize
    path_dx /= path_len;
    path_dy /= path_len;
    
    // Perpendicular direction (try both left and right)
    double perp_dx = -path_dy;
    double perp_dy = path_dx;
    
    double collision_radius = robot_radius_ + safety_margin_;
    
    // Try adjustments on both sides
    for (double offset = adjustment_step_; offset <= max_adjustment_; offset += adjustment_step_) {
      // Try left
      double new_x_left = original.pose.position.x + perp_dx * offset;
      double new_y_left = original.pose.position.y + perp_dy * offset;
      
      if (!checkObstacleNearPoint(new_x_left, new_y_left, collision_radius)) {
        adjusted.pose.position.x = new_x_left;
        adjusted.pose.position.y = new_y_left;
        return adjusted;
      }
      
      // Try right
      double new_x_right = original.pose.position.x - perp_dx * offset;
      double new_y_right = original.pose.position.y - perp_dy * offset;
      
      if (!checkObstacleNearPoint(new_x_right, new_y_right, collision_radius)) {
        adjusted.pose.position.x = new_x_right;
        adjusted.pose.position.y = new_y_right;
        return adjusted;
      }
    }
    
    // Could not find clear path - return original and log warning
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Could not find obstacle-free adjustment for waypoint at (%.2f, %.2f)",
                         original.pose.position.x, original.pose.position.y);
    
    return adjusted;
  }
  
  // Parameters
  bool enabled_;
  double target_height_;
  double height_tolerance_;
  double robot_radius_;
  double safety_margin_;
  double check_distance_;
  double adjustment_step_;
  double max_adjustment_;
  int min_points_for_obstacle_;
  double voxel_size_;
  std::string world_frame_;
  
  // State
  geometry_msgs::msg::Pose current_pose_;
  std::vector<geometry_msgs::msg::Point> obstacle_points_;
  bool have_odom_ = false;
  bool have_cloud_ = false;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr adjusted_path_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPathAdjusterNode>());
  rclcpp::shutdown();
  return 0;
}
