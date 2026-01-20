/**
 * Node: Telemetry Node
 *
 * Input:
 *   /odom (nav_msgs/Odometry) - Position, velocity
 *   /exploration/global_tour (ExplorationStatus) - Target info
 *   /exploration/trajectory (Trajectory) - Waypoint list
 *
 * Output:
 *   /exploration/telemetry (TelemetryStatus) - Real-time telemetry (30Hz)
 *
 * Features:
 *   - Instant/average velocity calculation
 *   - Acceleration computation (numerical differentiation)
 *   - Path length accumulation
 *   - CSV logging (optional)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "exploration_planner/msg/exploration_status.hpp"
#include "exploration_planner/msg/trajectory.hpp"
#include "exploration_planner/msg/telemetry_status.hpp"
#include "exploration_planner/common.hpp"

using namespace exploration_planner;

class TelemetryNode : public rclcpp::Node
{
public:
  TelemetryNode() : Node("telemetry_node")
  {
    // Parameters
    declare_parameter("publish_rate", 30.0);
    declare_parameter("logging_enabled", false);
    declare_parameter("log_file_path", "/tmp/exploration_telemetry.csv");

    publish_rate_ = get_parameter("publish_rate").as_double();
    logging_enabled_ = get_parameter("logging_enabled").as_bool();
    log_file_path_ = get_parameter("log_file_path").as_string();

    // Subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TelemetryNode::odomCallback, this, std::placeholders::_1));

    tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10,
      std::bind(&TelemetryNode::tourCallback, this, std::placeholders::_1));

    trajectory_sub_ = create_subscription<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10,
      std::bind(&TelemetryNode::trajectoryCallback, this, std::placeholders::_1));

    // Publisher
    telemetry_pub_ = create_publisher<exploration_planner::msg::TelemetryStatus>(
      "/exploration/telemetry", 10);

    // Timer for publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TelemetryNode::publishTelemetry, this));

    // Initialize CSV logging
    if (logging_enabled_) {
      initCSVLog();
    }

    RCLCPP_INFO(get_logger(), "Telemetry Node initialized");
    RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(get_logger(), "  CSV logging: %s", logging_enabled_ ? "enabled" : "disabled");
    if (logging_enabled_) {
      RCLCPP_INFO(get_logger(), "  Log file: %s", log_file_path_.c_str());
    }
  }

  ~TelemetryNode()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  void initCSVLog()
  {
    csv_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
      csv_file_ << "timestamp,vel_mag,vel_avg,accel_mag,yaw,yaw_error,dist_to_goal,"
                << "waypoint_idx,total_waypoints,path_traveled,session_time\n";
      csv_file_.flush();
      RCLCPP_INFO(get_logger(), "CSV log file created: %s", log_file_path_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to create CSV log file: %s", log_file_path_.c_str());
      logging_enabled_ = false;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto now_time = now();

    // Store previous values for acceleration calculation
    prev_velocity_ = current_velocity_;
    prev_time_ = current_time_;

    current_time_ = now_time;
    current_position_ = msg->pose.pose.position;
    current_orientation_ = msg->pose.pose.orientation;
    current_velocity_ = msg->twist.twist;

    // Calculate current velocity magnitude
    double vx = current_velocity_.linear.x;
    double vy = current_velocity_.linear.y;
    double vz = current_velocity_.linear.z;
    current_vel_magnitude_ = std::sqrt(vx*vx + vy*vy + vz*vz);
    current_vel_horizontal_ = std::sqrt(vx*vx + vy*vy);

    // Update average velocity (exponential moving average)
    if (!first_odom_received_) {
      first_odom_received_ = true;
      session_start_time_ = now_time;
      avg_velocity_ = current_vel_horizontal_;
      last_position_ = current_position_;
    } else {
      // EMA with alpha = 0.02 for smooth average
      double alpha = 0.02;
      avg_velocity_ = alpha * current_vel_horizontal_ + (1.0 - alpha) * avg_velocity_;

      // Accumulate path traveled
      double dx = current_position_.x - last_position_.x;
      double dy = current_position_.y - last_position_.y;
      double dz = current_position_.z - last_position_.z;
      double dist_delta = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist_delta < 1.0) {  // Sanity check - skip teleports
        total_path_traveled_ += dist_delta;
      }
      last_position_ = current_position_;
    }

    // Calculate acceleration (numerical differentiation)
    if (prev_time_.nanoseconds() > 0) {
      double dt = (current_time_ - prev_time_).seconds();
      if (dt > 0.001 && dt < 1.0) {  // Sanity check
        double dvx = current_velocity_.linear.x - prev_velocity_.linear.x;
        double dvy = current_velocity_.linear.y - prev_velocity_.linear.y;
        double dvz = current_velocity_.linear.z - prev_velocity_.linear.z;
        double accel = std::sqrt(dvx*dvx + dvy*dvy + dvz*dvz) / dt;

        // Low-pass filter for acceleration
        double alpha_accel = 0.3;
        current_acceleration_ = alpha_accel * accel + (1.0 - alpha_accel) * current_acceleration_;
      }
    }

    have_odom_ = true;
  }

  void tourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    current_tour_ = msg;

    // Extract target info
    if (!msg->waypoints.empty()) {
      target_position_ = msg->current_target.pose.position;
      target_yaw_ = getYaw(msg->current_target.pose.orientation);
      target_coverage_ = msg->target_coverage;
      target_cluster_id_ = msg->target_cluster_id;
    }

    current_waypoint_index_ = msg->current_waypoint_index;
    total_waypoints_ = msg->total_waypoints;
  }

  void trajectoryCallback(const exploration_planner::msg::Trajectory::SharedPtr msg)
  {
    current_trajectory_ = msg;

    // Update waypoint count from trajectory if available
    if (!msg->poses.empty()) {
      total_waypoints_ = msg->poses.size();
    }
  }

  void publishTelemetry()
  {
    if (!have_odom_) return;

    exploration_planner::msg::TelemetryStatus telemetry;
    telemetry.header.stamp = now();
    telemetry.header.frame_id = "map";

    // Velocity
    telemetry.velocity_magnitude = current_vel_magnitude_;
    telemetry.velocity_horizontal = current_vel_horizontal_;
    telemetry.average_velocity = avg_velocity_;

    // Acceleration
    telemetry.acceleration_magnitude = current_acceleration_;

    // Yaw
    double current_yaw = getYaw(current_orientation_);
    telemetry.current_yaw = current_yaw;
    telemetry.target_yaw = target_yaw_;
    telemetry.yaw_error = angleDiff(current_yaw, target_yaw_);

    // Target viewpoint
    telemetry.target_position = target_position_;
    telemetry.target_coverage = target_coverage_;
    telemetry.target_cluster_id = target_cluster_id_;

    // Progress
    telemetry.current_waypoint_index = current_waypoint_index_;
    telemetry.total_waypoints = total_waypoints_;
    telemetry.distance_to_goal = distance2D(current_position_, target_position_);

    // Path statistics
    telemetry.total_path_traveled = total_path_traveled_;
    telemetry.session_time = (now() - session_start_time_).seconds();

    telemetry_pub_->publish(telemetry);

    // CSV logging
    if (logging_enabled_ && csv_file_.is_open()) {
      csv_file_ << std::fixed << std::setprecision(3)
                << telemetry.header.stamp.sec << "." << telemetry.header.stamp.nanosec << ","
                << telemetry.velocity_magnitude << ","
                << telemetry.average_velocity << ","
                << telemetry.acceleration_magnitude << ","
                << telemetry.current_yaw << ","
                << telemetry.yaw_error << ","
                << telemetry.distance_to_goal << ","
                << telemetry.current_waypoint_index << ","
                << telemetry.total_waypoints << ","
                << telemetry.total_path_traveled << ","
                << telemetry.session_time << "\n";

      // Flush periodically (every ~10 seconds)
      static int flush_counter = 0;
      if (++flush_counter >= static_cast<int>(publish_rate_ * 10)) {
        csv_file_.flush();
        flush_counter = 0;
      }
    }
  }

  // Parameters
  double publish_rate_;
  bool logging_enabled_;
  std::string log_file_path_;

  // State
  bool have_odom_ = false;
  bool first_odom_received_ = false;
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point last_position_;
  geometry_msgs::msg::Quaternion current_orientation_;
  geometry_msgs::msg::Twist current_velocity_;
  geometry_msgs::msg::Twist prev_velocity_;
  rclcpp::Time current_time_;
  rclcpp::Time prev_time_;
  rclcpp::Time session_start_time_;

  double current_vel_magnitude_ = 0.0;
  double current_vel_horizontal_ = 0.0;
  double avg_velocity_ = 0.0;
  double current_acceleration_ = 0.0;
  double total_path_traveled_ = 0.0;

  // Target info
  geometry_msgs::msg::Point target_position_;
  double target_yaw_ = 0.0;
  double target_coverage_ = 0.0;
  uint32_t target_cluster_id_ = 0;
  uint32_t current_waypoint_index_ = 0;
  uint32_t total_waypoints_ = 0;

  // Tour/trajectory data
  exploration_planner::msg::ExplorationStatus::SharedPtr current_tour_;
  exploration_planner::msg::Trajectory::SharedPtr current_trajectory_;

  // CSV file
  std::ofstream csv_file_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr tour_sub_;
  rclcpp::Subscription<exploration_planner::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<exploration_planner::msg::TelemetryStatus>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
