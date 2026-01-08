/**
 * Node 5: Exploration Visualizer
 * 
 * Input:  /exploration/global_tour
 *         /exploration/refined_tour
 *         /exploration/trajectory
 * Output: /exploration/markers (visualization_msgs/MarkerArray)
 * 
 * Visualizes the exploration plan in RViz.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "exploration_planner/msg/exploration_status.hpp"
#include "exploration_planner/msg/trajectory.hpp"
#include "exploration_planner/common.hpp"

using namespace exploration_planner;

class ExplorationVisualizerNode : public rclcpp::Node
{
public:
  ExplorationVisualizerNode() : Node("exploration_visualizer")
  {
    // Subscribers
    global_tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10,
      std::bind(&ExplorationVisualizerNode::globalTourCallback, this, std::placeholders::_1));
    
    refined_tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/refined_tour", 10,
      std::bind(&ExplorationVisualizerNode::refinedTourCallback, this, std::placeholders::_1));
    
    trajectory_sub_ = create_subscription<exploration_planner::msg::Trajectory>(
      "/exploration/trajectory", 10,
      std::bind(&ExplorationVisualizerNode::trajectoryCallback, this, std::placeholders::_1));
    
    // Publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/exploration/markers", 10);
    
    RCLCPP_INFO(get_logger(), "Exploration Visualizer initialized");
  }

private:
  void globalTourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    global_tour_ = msg;
    publishMarkers();
  }
  
  void refinedTourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    refined_tour_ = msg;
    publishMarkers();
  }
  
  void trajectoryCallback(const exploration_planner::msg::Trajectory::SharedPtr msg)
  {
    current_trajectory_ = msg;
    publishMarkers();
  }
  
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    
    // Clear previous
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    
    int id = 0;
    std::string frame = "map";
    
    if (global_tour_) {
      frame = global_tour_->header.frame_id;
    }
    
    auto stamp = now();
    
    // === Global Tour (dashed line, yellow) ===
    if (global_tour_ && !global_tour_->waypoints.empty()) {
      visualization_msgs::msg::Marker tour_line;
      tour_line.header.frame_id = frame;
      tour_line.header.stamp = stamp;
      tour_line.ns = "global_tour";
      tour_line.id = id++;
      tour_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      tour_line.action = visualization_msgs::msg::Marker::ADD;
      tour_line.scale.x = 0.08;
      tour_line.color.r = 1.0;
      tour_line.color.g = 1.0;
      tour_line.color.b = 0.0;
      tour_line.color.a = 0.6;
      
      for (const auto& wp : global_tour_->waypoints) {
        geometry_msgs::msg::Point p = wp.pose.position;
        p.z += 0.1;
        tour_line.points.push_back(p);
      }
      markers.markers.push_back(tour_line);
      
      // Waypoint spheres
      for (size_t i = 0; i < global_tour_->waypoints.size(); ++i) {
        visualization_msgs::msg::Marker sphere;
        sphere.header = tour_line.header;
        sphere.ns = "global_waypoints";
        sphere.id = id++;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose = global_tour_->waypoints[i].pose;
        sphere.pose.position.z += 0.1;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3;
        
        // Current target is larger and different color
        if (i == global_tour_->current_waypoint_index) {
          sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5;
          sphere.color.r = 0.0;
          sphere.color.g = 1.0;
          sphere.color.b = 0.0;
          sphere.color.a = 1.0;
        } else {
          sphere.color.r = 1.0;
          sphere.color.g = 0.8;
          sphere.color.b = 0.0;
          sphere.color.a = 0.7;
        }
        markers.markers.push_back(sphere);
        
        // Number label
        visualization_msgs::msg::Marker text;
        text.header = tour_line.header;
        text.ns = "waypoint_labels";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position = global_tour_->waypoints[i].pose.position;
        text.pose.position.z += 0.5;
        text.scale.z = 0.3;
        text.color.r = text.color.g = text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = std::to_string(i + 1);
        markers.markers.push_back(text);
      }
    }
    
    // === Refined Tour (solid line, cyan) ===
    if (refined_tour_ && !refined_tour_->waypoints.empty()) {
      visualization_msgs::msg::Marker refined_line;
      refined_line.header.frame_id = frame;
      refined_line.header.stamp = stamp;
      refined_line.ns = "refined_tour";
      refined_line.id = id++;
      refined_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      refined_line.action = visualization_msgs::msg::Marker::ADD;
      refined_line.scale.x = 0.06;
      refined_line.color.r = 0.0;
      refined_line.color.g = 1.0;
      refined_line.color.b = 1.0;
      refined_line.color.a = 0.8;
      
      for (const auto& wp : refined_tour_->waypoints) {
        geometry_msgs::msg::Point p = wp.pose.position;
        p.z += 0.15;
        refined_line.points.push_back(p);
      }
      markers.markers.push_back(refined_line);
    }
    
    // === Current Trajectory (green line with velocity arrows) ===
    if (current_trajectory_ && !current_trajectory_->poses.empty()) {
      visualization_msgs::msg::Marker traj_line;
      traj_line.header.frame_id = frame;
      traj_line.header.stamp = stamp;
      traj_line.ns = "trajectory";
      traj_line.id = id++;
      traj_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      traj_line.action = visualization_msgs::msg::Marker::ADD;
      traj_line.scale.x = 0.04;
      traj_line.color.r = 0.0;
      traj_line.color.g = 1.0;
      traj_line.color.b = 0.3;
      traj_line.color.a = 1.0;
      
      for (const auto& pose : current_trajectory_->poses) {
        geometry_msgs::msg::Point p = pose.pose.position;
        p.z += 0.2;
        traj_line.points.push_back(p);
      }
      markers.markers.push_back(traj_line);
      
      // Velocity arrows (every few points)
      int step = std::max(1, static_cast<int>(current_trajectory_->poses.size() / 10));
      for (size_t i = 0; i < current_trajectory_->poses.size() && 
                         i < current_trajectory_->velocities.size(); i += step) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = traj_line.header;
        arrow.ns = "velocity_arrows";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        
        const auto& vel = current_trajectory_->velocities[i];
        double vel_mag = std::sqrt(vel.linear.x*vel.linear.x + vel.linear.y*vel.linear.y);
        
        if (vel_mag < 0.01) continue;
        
        geometry_msgs::msg::Point start, end;
        start = current_trajectory_->poses[i].pose.position;
        start.z += 0.2;
        end.x = start.x + vel.linear.x * 0.5;  // Scale for visibility
        end.y = start.y + vel.linear.y * 0.5;
        end.z = start.z;
        
        arrow.points = {start, end};
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.06;
        arrow.scale.z = 0.06;
        arrow.color.r = 0.3;
        arrow.color.g = 1.0;
        arrow.color.b = 0.3;
        arrow.color.a = 0.8;
        markers.markers.push_back(arrow);
      }
    }
    
    // === Status Text ===
    if (global_tour_) {
      visualization_msgs::msg::Marker status_text;
      status_text.header.frame_id = frame;
      status_text.header.stamp = stamp;
      status_text.ns = "status";
      status_text.id = id++;
      status_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      status_text.action = visualization_msgs::msg::Marker::ADD;
      status_text.pose.position.x = 0;
      status_text.pose.position.y = 0;
      status_text.pose.position.z = 3.0;
      status_text.scale.z = 0.4;
      status_text.color.r = status_text.color.g = status_text.color.b = 1.0;
      status_text.color.a = 1.0;
      
      std::string state_str;
      switch (global_tour_->state) {
        case exploration_planner::msg::ExplorationStatus::IDLE: state_str = "IDLE"; break;
        case exploration_planner::msg::ExplorationStatus::EXPLORING: state_str = "EXPLORING"; break;
        case exploration_planner::msg::ExplorationStatus::MOVING_TO_VIEWPOINT: state_str = "MOVING"; break;
        case exploration_planner::msg::ExplorationStatus::REFINING: state_str = "REFINING"; break;
        case exploration_planner::msg::ExplorationStatus::COMPLETED: state_str = "COMPLETED"; break;
        default: state_str = "UNKNOWN"; break;
      }
      
      std::stringstream ss;
      ss << "State: " << state_str << "\n";
      ss << "Waypoint: " << (global_tour_->current_waypoint_index + 1) 
         << "/" << global_tour_->total_waypoints << "\n";
      ss << std::fixed << std::setprecision(1);
      ss << "Est. time: " << global_tour_->estimated_time_remaining << "s\n";
      ss << "Remaining: " << global_tour_->total_distance_remaining << "m";
      
      status_text.text = ss.str();
      markers.markers.push_back(status_text);
    }
    
    marker_pub_->publish(markers);
  }
  
  exploration_planner::msg::ExplorationStatus::SharedPtr global_tour_;
  exploration_planner::msg::ExplorationStatus::SharedPtr refined_tour_;
  exploration_planner::msg::Trajectory::SharedPtr current_trajectory_;
  
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr global_tour_sub_;
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr refined_tour_sub_;
  rclcpp::Subscription<exploration_planner::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}