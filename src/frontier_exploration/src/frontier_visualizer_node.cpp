/**
 * Node 4: Frontier Visualizer
 * 
 * Input:  /frontier_clusters_complete (frontier_exploration/FrontierArray)
 * Output: /frontier_markers (visualization_msgs/MarkerArray)
 *         /fis_info (std_msgs/String) - human-readable FIS info
 * 
 * Visualizes clusters, viewpoints, and connections in RViz.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "frontier_exploration/msg/frontier_array.hpp"
#include "frontier_exploration/common.hpp"

#include <sstream>
#include <iomanip>

using namespace frontier_exploration;

class FrontierVisualizerNode : public rclcpp::Node
{
public:
  FrontierVisualizerNode() : Node("frontier_visualizer")
  {
    // Subscriber
    clusters_sub_ = create_subscription<frontier_exploration::msg::FrontierArray>(
      "frontier_clusters_complete", 10,
      std::bind(&FrontierVisualizerNode::clustersCallback, this, std::placeholders::_1));
    
    // Publishers
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "frontier_markers", 10);
    info_pub_ = create_publisher<std_msgs::msg::String>("fis_info", 10);
    
    RCLCPP_INFO(get_logger(), "Frontier Visualizer initialized");
    RCLCPP_INFO(get_logger(), "  Input:  frontier_clusters_complete");
    RCLCPP_INFO(get_logger(), "  Output: frontier_markers, fis_info");
  }

private:
  void clustersCallback(const frontier_exploration::msg::FrontierArray::SharedPtr msg)
  {
    publishMarkers(msg);
    publishInfo(msg);
  }
  
  void publishMarkers(const frontier_exploration::msg::FrontierArray::SharedPtr& msg)
  {
    visualization_msgs::msg::MarkerArray markers;
    
    // Clear previous
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    
    int id = 0;
    std::string frame = msg->header.frame_id;
    auto stamp = msg->header.stamp;
    
    for (const auto& cluster : msg->clusters) {
      // === Centroid sphere ===
      visualization_msgs::msg::Marker centroid;
      centroid.header.frame_id = frame;
      centroid.header.stamp = stamp;
      centroid.ns = "centroids";
      centroid.id = id++;
      centroid.type = visualization_msgs::msg::Marker::SPHERE;
      centroid.action = visualization_msgs::msg::Marker::ADD;
      centroid.pose.position = cluster.centroid;
      centroid.pose.position.z = 0.3;
      centroid.pose.orientation.w = 1.0;
      centroid.scale.x = centroid.scale.y = centroid.scale.z = 0.4;
      centroid.color = cluster.color;
      centroid.color.a = 1.0;
      markers.markers.push_back(centroid);
      
      // === Principal axis arrow ===
      visualization_msgs::msg::Marker axis;
      axis.header = centroid.header;
      axis.ns = "axes";
      axis.id = id++;
      axis.type = visualization_msgs::msg::Marker::ARROW;
      axis.action = visualization_msgs::msg::Marker::ADD;
      
      geometry_msgs::msg::Point start, end;
      start = cluster.centroid;
      start.z = 0.2;
      double len = std::min(std::max(std::sqrt(cluster.principal_eigenvalue) * 0.5, 0.5), 2.0);
      end.x = start.x + cluster.principal_axis[0] * len;
      end.y = start.y + cluster.principal_axis[1] * len;
      end.z = 0.2;
      
      axis.points = {start, end};
      axis.scale.x = 0.08;
      axis.scale.y = 0.15;
      axis.scale.z = 0.15;
      axis.color = cluster.color;
      axis.color.a = 0.8;
      markers.markers.push_back(axis);
      
      // === Frontier cells (colored points) ===
      if (!cluster.cells.empty()) {
        visualization_msgs::msg::Marker cells_marker;
        cells_marker.header = centroid.header;
        cells_marker.ns = "frontier_cells";
        cells_marker.id = id++;
        cells_marker.type = visualization_msgs::msg::Marker::POINTS;
        cells_marker.action = visualization_msgs::msg::Marker::ADD;
        cells_marker.scale.x = 0.15;  // Point size
        cells_marker.scale.y = 0.15;
        cells_marker.color = cluster.color;
        cells_marker.color.a = 0.7;
        
        for (const auto& cell : cluster.cells) {
          geometry_msgs::msg::Point p = cell;
          p.z = 0.05;  // Slightly above ground
          cells_marker.points.push_back(p);
        }
        markers.markers.push_back(cells_marker);
      }
      
      // === Viewpoints ===
      for (size_t vi = 0; vi < cluster.viewpoints.size(); ++vi) {
        const auto& vp = cluster.viewpoints[vi];
        bool is_best = (vi == 0);
        
        // Position cube
        visualization_msgs::msg::Marker vp_marker;
        vp_marker.header = centroid.header;
        vp_marker.ns = "viewpoints";
        vp_marker.id = id++;
        vp_marker.type = visualization_msgs::msg::Marker::CUBE;
        vp_marker.action = visualization_msgs::msg::Marker::ADD;
        vp_marker.pose.position = vp.position;
        vp_marker.pose.position.z = 0.5;
        vp_marker.pose.orientation.z = std::sin(vp.yaw / 2.0);
        vp_marker.pose.orientation.w = std::cos(vp.yaw / 2.0);
        
        double scale = is_best ? 0.3 : 0.15;
        vp_marker.scale.x = scale;
        vp_marker.scale.y = scale;
        vp_marker.scale.z = scale * 0.5;
        vp_marker.color = cluster.color;
        vp_marker.color.a = is_best ? 1.0 : 0.5;
        markers.markers.push_back(vp_marker);
        
        // FOV arrow
        visualization_msgs::msg::Marker fov;
        fov.header = centroid.header;
        fov.ns = "fov";
        fov.id = id++;
        fov.type = visualization_msgs::msg::Marker::ARROW;
        fov.action = visualization_msgs::msg::Marker::ADD;
        
        start = vp.position;
        start.z = 0.5;
        double arrow_len = is_best ? 1.5 : 0.8;
        end.x = start.x + std::cos(vp.yaw) * arrow_len;
        end.y = start.y + std::sin(vp.yaw) * arrow_len;
        end.z = 0.5;
        
        fov.points = {start, end};
        fov.scale.x = 0.06;
        fov.scale.y = 0.12;
        fov.scale.z = 0.12;
        fov.color = cluster.color;
        fov.color.a = 0.7;
        markers.markers.push_back(fov);
        
        // Coverage text for best
        if (is_best) {
          visualization_msgs::msg::Marker text;
          text.header = centroid.header;
          text.ns = "vp_info";
          text.id = id++;
          text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          text.action = visualization_msgs::msg::Marker::ADD;
          text.pose.position = vp.position;
          text.pose.position.z = 1.0;
          text.scale.z = 0.2;
          text.color = createColor(1, 1, 1, 1);
          text.text = "cov:" + std::to_string(vp.coverage);
          markers.markers.push_back(text);
        }
      }
      
      // === Connection lines ===
      for (const auto& conn : cluster.connections) {
        // Only draw if our ID is smaller
        if (cluster.id >= conn.target_cluster_id) continue;
        
        // Find target
        const frontier_exploration::msg::FrontierCluster* target = nullptr;
        for (const auto& c : msg->clusters) {
          if (c.id == conn.target_cluster_id) {
            target = &c;
            break;
          }
        }
        
        if (!target || cluster.viewpoints.empty() || target->viewpoints.empty()) continue;
        
        visualization_msgs::msg::Marker line;
        line.header = centroid.header;
        line.ns = "connections";
        line.id = id++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.03;
        
        geometry_msgs::msg::Point p1 = cluster.viewpoints[0].position;
        geometry_msgs::msg::Point p2 = target->viewpoints[0].position;
        p1.z = p2.z = 0.4;
        line.points = {p1, p2};
        line.color = createColor(0.7, 0.7, 0.7, 0.4);
        markers.markers.push_back(line);
        
        // Cost label
        visualization_msgs::msg::Marker cost_text;
        cost_text.header = centroid.header;
        cost_text.ns = "costs";
        cost_text.id = id++;
        cost_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        cost_text.action = visualization_msgs::msg::Marker::ADD;
        cost_text.pose.position.x = (p1.x + p2.x) / 2;
        cost_text.pose.position.y = (p1.y + p2.y) / 2;
        cost_text.pose.position.z = 0.7;
        cost_text.scale.z = 0.15;
        cost_text.color = createColor(0.9, 0.9, 0.9, 0.8);
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << conn.cost << "s";
        cost_text.text = ss.str();
        markers.markers.push_back(cost_text);
      }
      
      // === Cluster label ===
      visualization_msgs::msg::Marker label;
      label.header = centroid.header;
      label.ns = "labels";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = cluster.centroid;
      label.pose.position.z = 0.8;
      label.scale.z = 0.25;
      label.color = createColor(1, 1, 1, 1);
      label.text = "C" + std::to_string(cluster.id) + " (" + std::to_string(cluster.size) + ")";
      markers.markers.push_back(label);
    }
    
    marker_pub_->publish(markers);
  }
  
  void publishInfo(const frontier_exploration::msg::FrontierArray::SharedPtr& msg)
  {
    std_msgs::msg::String info;
    std::stringstream ss;
    
    ss << "=== Frontier Information Structures ===\n";
    ss << "Timestamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << "\n";
    ss << "Frame: " << msg->header.frame_id << "\n";
    ss << "Total clusters: " << msg->clusters.size() << "\n\n";
    
    for (const auto& cluster : msg->clusters) {
      ss << "--- Cluster " << cluster.id << " ---\n";
      ss << "  Size: " << cluster.size << " cells\n";
      ss << std::fixed << std::setprecision(2);
      ss << "  Centroid: (" << cluster.centroid.x << ", " << cluster.centroid.y << ")\n";
      ss << "  BBOX: [" << cluster.bbox_min_x << "," << cluster.bbox_min_y 
         << "] -> [" << cluster.bbox_max_x << "," << cluster.bbox_max_y << "]\n";
      ss << "  Principal eigenvalue: " << cluster.principal_eigenvalue << "\n";
      ss << "  Viewpoints: " << cluster.viewpoints.size() << "\n";
      
      for (size_t i = 0; i < cluster.viewpoints.size(); ++i) {
        const auto& vp = cluster.viewpoints[i];
        ss << "    VP" << i << ": (" << vp.position.x << ", " << vp.position.y 
           << ") yaw=" << std::setprecision(0) << (vp.yaw * 180.0 / M_PI) 
           << "° cov=" << vp.coverage << "\n";
        ss << std::setprecision(2);
      }
      
      ss << "  Connections: " << cluster.connections.size() << "\n";
      for (const auto& conn : cluster.connections) {
        ss << "    -> C" << conn.target_cluster_id 
           << ": cost=" << conn.cost << "s dist=" << conn.path_length << "m\n";
      }
      ss << "\n";
    }
    
    info.data = ss.str();
    info_pub_->publish(info);
  }
  
  rclcpp::Subscription<frontier_exploration::msg::FrontierArray>::SharedPtr clusters_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}