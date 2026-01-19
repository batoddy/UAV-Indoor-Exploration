/**
 * Nav2 Path Planner Node
 *
 * Calls Nav2 planner server to compute collision-free paths.
 *
 * Input:  /exploration/global_tour (ExplorationStatus) - target waypoint
 *         /odom - current position
 * Output: /exploration/nav2_path (nav_msgs/Path) - computed path
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "exploration_planner/msg/exploration_status.hpp"

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandlePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class Nav2PathPlannerNode : public rclcpp::Node
{
public:
  Nav2PathPlannerNode() : Node("nav2_path_planner")
  {
    declare_parameter("planner_id", "GridBased");
    declare_parameter("goal_frame", "map");
    declare_parameter("replan_rate", 2.0);

    planner_id_ = get_parameter("planner_id").as_string();
    goal_frame_ = get_parameter("goal_frame").as_string();
    double replan_rate = get_parameter("replan_rate").as_double();

    // Action client
    action_client_ = rclcpp_action::create_client<ComputePathToPose>(
      this, "compute_path_to_pose");

    // Subscribers
    tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10,
      std::bind(&Nav2PathPlannerNode::tourCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&Nav2PathPlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/nav2_path", 10);

    // Replan timer
    replan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / replan_rate),
      std::bind(&Nav2PathPlannerNode::replanTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Nav2 Path Planner initialized");
    RCLCPP_INFO(get_logger(), "  Planner: %s, Frame: %s", planner_id_.c_str(), goal_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    have_pose_ = true;
  }

  void tourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    if (msg->waypoints.empty()) {
      has_goal_ = false;
      return;
    }

    if (msg->state == exploration_planner::msg::ExplorationStatus::COMPLETED ||
        msg->state == exploration_planner::msg::ExplorationStatus::IDLE) {
      has_goal_ = false;
      return;
    }

    current_goal_ = msg->current_target;
    has_goal_ = true;
    needs_replan_ = true;

    RCLCPP_DEBUG(get_logger(), "New goal: (%.2f, %.2f)",
                 current_goal_.pose.position.x, current_goal_.pose.position.y);
  }

  void replanTimerCallback()
  {
    if (!has_goal_ || !have_pose_ || !needs_replan_) {
      return;
    }

    if (planning_in_progress_) {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for Nav2 planner server...");
      return;
    }

    requestPath();
    needs_replan_ = false;
  }

  void requestPath()
  {
    auto goal_msg = ComputePathToPose::Goal();
    goal_msg.goal = current_goal_;
    goal_msg.goal.header.frame_id = goal_frame_;
    goal_msg.goal.header.stamp = now();
    goal_msg.planner_id = planner_id_;
    goal_msg.start = current_pose_;
    goal_msg.start.header.frame_id = goal_frame_;
    goal_msg.start.header.stamp = now();
    goal_msg.use_start = true;

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&Nav2PathPlannerNode::resultCallback, this, std::placeholders::_1);

    planning_in_progress_ = true;
    action_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_DEBUG(get_logger(), "Requested path from (%.2f, %.2f) to (%.2f, %.2f)",
                 current_pose_.pose.position.x, current_pose_.pose.position.y,
                 current_goal_.pose.position.x, current_goal_.pose.position.y);
  }

  void resultCallback(const GoalHandlePath::WrappedResult& result)
  {
    planning_in_progress_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Path planning aborted");
        needs_replan_ = true;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Path planning canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }

    if (result.result->path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path");
      needs_replan_ = true;
      return;
    }

    // Preserve target yaw in the last pose
    nav_msgs::msg::Path path = result.result->path;
    if (!path.poses.empty()) {
      path.poses.back().pose.orientation = current_goal_.pose.orientation;
    }

    path_pub_->publish(path);

    RCLCPP_DEBUG(get_logger(), "Published path with %zu poses", path.poses.size());
  }

  // Parameters
  std::string planner_id_;
  std::string goal_frame_;

  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped current_goal_;
  bool have_pose_ = false;
  bool has_goal_ = false;
  bool needs_replan_ = false;
  bool planning_in_progress_ = false;

  // ROS
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr tour_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
