/**
 * Exploration Visualizer
 * 
 * Tüm exploration verilerini RViz için görselleştirir:
 * - Frontier clusters (points + centroids)
 * - Current goal
 * - Exploration state
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <frontier_interfaces/msg/frontier_table.hpp>
#include <frontier_interfaces/msg/exploration_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ExplorationVisualizerNode : public rclcpp::Node {
public:
    ExplorationVisualizerNode() : Node("exploration_visualizer") {
        // Subscribers
        table_sub_ = create_subscription<frontier_interfaces::msg::FrontierTable>(
            "/frontier_table", 10,
            std::bind(&ExplorationVisualizerNode::tableCallback, this, std::placeholders::_1));
        
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/selected_goal", 10,
            std::bind(&ExplorationVisualizerNode::goalCallback, this, std::placeholders::_1));
        
        state_sub_ = create_subscription<frontier_interfaces::msg::ExplorationState>(
            "/exploration_state", 10,
            std::bind(&ExplorationVisualizerNode::stateCallback, this, std::placeholders::_1));
        
        // Publisher
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/exploration_markers", 10);
        
        RCLCPP_INFO(get_logger(), "Exploration Visualizer started");
    }

private:
    void tableCallback(const frontier_interfaces::msg::FrontierTable::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray markers;
        
        // Clear previous markers
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);
        
        int id = 0;
        
        for (const auto& cluster : msg->clusters) {
            // Cluster points (green)
            visualization_msgs::msg::Marker points;
            points.header = msg->header;
            points.ns = "frontier_points";
            points.id = id++;
            points.type = visualization_msgs::msg::Marker::POINTS;
            points.action = visualization_msgs::msg::Marker::ADD;
            points.scale.x = 0.08;
            points.scale.y = 0.08;
            
            // Color based on reachability
            if (cluster.reachable) {
                points.color.r = 0.0;
                points.color.g = 1.0;
                points.color.b = 0.0;
            } else {
                points.color.r = 0.5;
                points.color.g = 0.5;
                points.color.b = 0.5;
            }
            points.color.a = 0.7;
            
            for (const auto& p : cluster.points) {
                points.points.push_back(p);
            }
            markers.markers.push_back(points);
            
            // Centroid sphere (orange/yellow based on cost)
            visualization_msgs::msg::Marker centroid;
            centroid.header = msg->header;
            centroid.ns = "frontier_centroids";
            centroid.id = id++;
            centroid.type = visualization_msgs::msg::Marker::SPHERE;
            centroid.action = visualization_msgs::msg::Marker::ADD;
            centroid.pose.position = cluster.centroid;
            centroid.pose.orientation.w = 1.0;
            centroid.scale.x = 0.4;
            centroid.scale.y = 0.4;
            centroid.scale.z = 0.4;
            
            // Color: yellow (low cost) to red (high cost)
            double cost_norm = std::min(cluster.cost / 20.0, 1.0);
            centroid.color.r = cost_norm;
            centroid.color.g = 1.0 - cost_norm;
            centroid.color.b = 0.0;
            centroid.color.a = 0.9;
            markers.markers.push_back(centroid);
            
            // Text label
            visualization_msgs::msg::Marker text;
            text.header = msg->header;
            text.ns = "frontier_labels";
            text.id = id++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position = cluster.centroid;
            text.pose.position.z += 0.6;
            text.scale.z = 0.25;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            
            char buf[64];
            snprintf(buf, sizeof(buf), "#%u c:%.1f s:%u", 
                cluster.id, cluster.cost, cluster.size);
            text.text = buf;
            markers.markers.push_back(text);
        }
        
        marker_pub_->publish(markers);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray markers;
        
        // Goal marker (magenta arrow)
        visualization_msgs::msg::Marker goal;
        goal.header = msg->header;
        goal.ns = "current_goal";
        goal.id = 0;
        goal.type = visualization_msgs::msg::Marker::ARROW;
        goal.action = visualization_msgs::msg::Marker::ADD;
        goal.pose = msg->pose;
        goal.scale.x = 1.0;
        goal.scale.y = 0.2;
        goal.scale.z = 0.2;
        goal.color.r = 1.0;
        goal.color.g = 0.0;
        goal.color.b = 1.0;
        goal.color.a = 1.0;
        markers.markers.push_back(goal);
        
        // Goal sphere
        visualization_msgs::msg::Marker sphere;
        sphere.header = msg->header;
        sphere.ns = "current_goal";
        sphere.id = 1;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position = msg->pose.position;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.6;
        sphere.scale.y = 0.6;
        sphere.scale.z = 0.6;
        sphere.color.r = 1.0;
        sphere.color.g = 0.0;
        sphere.color.b = 1.0;
        sphere.color.a = 0.5;
        markers.markers.push_back(sphere);
        
        marker_pub_->publish(markers);
    }
    
    void stateCallback(const frontier_interfaces::msg::ExplorationState::SharedPtr msg) {
        // Could add state visualization here (e.g., text display)
        RCLCPP_DEBUG(get_logger(), "State: %s", msg->status_message.c_str());
    }
    
    // ROS
    rclcpp::Subscription<frontier_interfaces::msg::FrontierTable>::SharedPtr table_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<frontier_interfaces::msg::ExplorationState>::SharedPtr state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorationVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
