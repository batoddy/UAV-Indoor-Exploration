#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_detection/frontier_detector.hpp"

class FrontierDetectorNode : public rclcpp::Node
{
public:
  FrontierDetectorNode() : Node("frontier_detector")
  {
    // Parameters
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("update_rate", 1.0);
    this->declare_parameter("min_frontier_size", 5);
    this->declare_parameter("max_cluster_size", 50);
    this->declare_parameter("free_threshold", 25);
    this->declare_parameter("occupied_threshold", 65);
    this->declare_parameter("pca_split_threshold", 2.0);
    
    std::string map_topic = this->get_parameter("map_topic").as_string();
    double update_rate = this->get_parameter("update_rate").as_double();
    int min_size = this->get_parameter("min_frontier_size").as_int();
    int max_size = this->get_parameter("max_cluster_size").as_int();
    int free_thresh = this->get_parameter("free_threshold").as_int();
    int occ_thresh = this->get_parameter("occupied_threshold").as_int();
    double pca_thresh = this->get_parameter("pca_split_threshold").as_double();
    
    // Configure detector
    detector_.setMinFrontierSize(min_size);
    detector_.setMaxClusterSize(max_size);
    detector_.setFreeThreshold(static_cast<int8_t>(free_thresh));
    detector_.setOccupiedThreshold(static_cast<int8_t>(occ_thresh));
    detector_.setPcaSplitThreshold(pca_thresh);
    
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10,
      std::bind(&FrontierDetectorNode::mapCallback, this, std::placeholders::_1));
    
    // Publishers
    frontiers_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "frontiers", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "frontier_markers", 10);
    
    // Timer for periodic detection
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate),
      std::bind(&FrontierDetectorNode::detectAndPublish, this));
    
    RCLCPP_INFO(this->get_logger(), "Frontier detector initialized");
    RCLCPP_INFO(this->get_logger(), "  Map topic: %s", map_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Min frontier size: %d", min_size);
    RCLCPP_INFO(this->get_logger(), "  Max cluster size: %d (PCA split threshold: %.2f)", max_size, pca_thresh);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    current_map_ = msg;
  }
  
  void detectAndPublish()
  {
    if (!current_map_) {
      return;
    }
    
    auto clusters = detector_.detectFrontiers(current_map_);
    
    RCLCPP_DEBUG(this->get_logger(), "Detected %zu frontier clusters", clusters.size());
    
    publishFrontiers(clusters);
    publishMarkers(clusters);
  }
  
  void publishFrontiers(const std::vector<frontier_detection::FrontierCluster>& clusters)
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = current_map_->header.frame_id;
    
    for (const auto& cluster : clusters) {
      geometry_msgs::msg::Pose pose;
      pose.position = cluster.centroid;
      
      // Orientation from principal axis
      double yaw = std::atan2(cluster.principal_axis[1], cluster.principal_axis[0]);
      pose.orientation.z = std::sin(yaw / 2.0);
      pose.orientation.w = std::cos(yaw / 2.0);
      
      msg.poses.push_back(pose);
    }
    
    frontiers_pub_->publish(msg);
  }
  
  void publishMarkers(const std::vector<frontier_detection::FrontierCluster>& clusters)
  {
    visualization_msgs::msg::MarkerArray markers;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    
    int id = 0;
    for (const auto& cluster : clusters) {
      // Centroid marker (sphere)
      visualization_msgs::msg::Marker centroid;
      centroid.header.stamp = this->now();
      centroid.header.frame_id = current_map_->header.frame_id;
      centroid.ns = "cluster_centroids";
      centroid.id = id++;
      centroid.type = visualization_msgs::msg::Marker::SPHERE;
      centroid.action = visualization_msgs::msg::Marker::ADD;
      centroid.pose.position = cluster.centroid;
      centroid.pose.position.z = 0.3;  // Lift slightly
      centroid.pose.orientation.w = 1.0;
      
      double scale = std::min(0.4 + cluster.size * 0.01, 1.0);
      centroid.scale.x = scale;
      centroid.scale.y = scale;
      centroid.scale.z = scale;
      
      // Use cluster's assigned color
      centroid.color = cluster.color;
      centroid.color.a = 1.0;
      
      markers.markers.push_back(centroid);
      
      // Principal axis arrow
      visualization_msgs::msg::Marker arrow;
      arrow.header = centroid.header;
      arrow.ns = "principal_axes";
      arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      
      geometry_msgs::msg::Point start, end;
      start = cluster.centroid;
      start.z = 0.2;
      
      double arrow_length = std::sqrt(cluster.principal_eigenvalue) * current_map_->info.resolution * 2.0;
      arrow_length = std::min(std::max(arrow_length, 0.5), 3.0);
      
      end.x = start.x + cluster.principal_axis[0] * arrow_length;
      end.y = start.y + cluster.principal_axis[1] * arrow_length;
      end.z = 0.2;
      
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.scale.x = 0.08;  // Shaft diameter
      arrow.scale.y = 0.15;  // Head diameter
      arrow.scale.z = 0.15;  // Head length
      
      arrow.color = cluster.color;
      arrow.color.a = 0.9;
      
      markers.markers.push_back(arrow);
      
      // Frontier cells (points) - same color as centroid
      visualization_msgs::msg::Marker cells;
      cells.header = centroid.header;
      cells.ns = "frontier_cells";
      cells.id = id++;
      cells.type = visualization_msgs::msg::Marker::POINTS;
      cells.action = visualization_msgs::msg::Marker::ADD;
      cells.scale.x = current_map_->info.resolution * 0.8;
      cells.scale.y = current_map_->info.resolution * 0.8;
      
      // Use cluster color for cells too
      cells.color = cluster.color;
      cells.color.a = 0.6;
      
      for (const auto& [gx, gy] : cluster.cells) {
        geometry_msgs::msg::Point p;
        p.x = current_map_->info.origin.position.x + (gx + 0.5) * current_map_->info.resolution;
        p.y = current_map_->info.origin.position.y + (gy + 0.5) * current_map_->info.resolution;
        p.z = 0.05;
        cells.points.push_back(p);
      }
      
      markers.markers.push_back(cells);
      
      // Bounding box
      visualization_msgs::msg::Marker bbox;
      bbox.header = centroid.header;
      bbox.ns = "bounding_boxes";
      bbox.id = id++;
      bbox.type = visualization_msgs::msg::Marker::LINE_STRIP;
      bbox.action = visualization_msgs::msg::Marker::ADD;
      bbox.scale.x = 0.03;
      
      bbox.color = cluster.color;
      bbox.color.a = 0.4;
      
      // Convert bbox grid coords to world
      double res = current_map_->info.resolution;
      double ox = current_map_->info.origin.position.x;
      double oy = current_map_->info.origin.position.y;
      
      geometry_msgs::msg::Point p1, p2, p3, p4;
      p1.x = ox + cluster.bbox.min_x * res; p1.y = oy + cluster.bbox.min_y * res; p1.z = 0.1;
      p2.x = ox + cluster.bbox.max_x * res; p2.y = oy + cluster.bbox.min_y * res; p2.z = 0.1;
      p3.x = ox + cluster.bbox.max_x * res; p3.y = oy + cluster.bbox.max_y * res; p3.z = 0.1;
      p4.x = ox + cluster.bbox.min_x * res; p4.y = oy + cluster.bbox.max_y * res; p4.z = 0.1;
      
      bbox.points = {p1, p2, p3, p4, p1};  // Close the loop
      
      markers.markers.push_back(bbox);
      
      // Text label with cluster ID and size
      visualization_msgs::msg::Marker text;
      text.header = centroid.header;
      text.ns = "cluster_labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = cluster.centroid;
      text.pose.position.z = 0.6;
      text.scale.z = 0.3;
      
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      
      text.text = "C" + std::to_string(cluster.id) + " (" + std::to_string(static_cast<int>(cluster.size)) + ")";
      
      markers.markers.push_back(text);
    }
    
    marker_pub_->publish(markers);
  }
  
  frontier_detection::FrontierDetector detector_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierDetectorNode>());
  rclcpp::shutdown();
  return 0;
}