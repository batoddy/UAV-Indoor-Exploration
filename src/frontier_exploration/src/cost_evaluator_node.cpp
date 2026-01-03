/**
 * Node 3: Cost Evaluator
 * 
 * Input:  /frontier_clusters (frontier_interfaces/FrontierTable)
 *         /odom (nav_msgs/Odometry)
 * Output: /frontier_table (frontier_interfaces/FrontierTable) - with costs
 * 
 * Her frontier cluster için cost hesaplar:
 * - Distance cost
 * - Size utility
 * - (Extensible: information gain, path cost, etc.)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <frontier_interfaces/msg/frontier_table.hpp>

#include <cmath>

class CostEvaluatorNode : public rclcpp::Node {
public:
    CostEvaluatorNode() : Node("cost_evaluator") {
        // Parameters - cost function weights
        declare_parameter("weight_distance", 1.0);
        declare_parameter("weight_size", 0.5);
        declare_parameter("weight_height_penalty", 0.3);
        declare_parameter("preferred_height", 2.0);
        declare_parameter("max_distance", 50.0);
        
        w_distance_ = get_parameter("weight_distance").as_double();
        w_size_ = get_parameter("weight_size").as_double();
        w_height_ = get_parameter("weight_height_penalty").as_double();
        preferred_height_ = get_parameter("preferred_height").as_double();
        max_distance_ = get_parameter("max_distance").as_double();
        
        // Subscribers
        clusters_sub_ = create_subscription<frontier_interfaces::msg::FrontierTable>(
            "/frontier_clusters", 10,
            std::bind(&CostEvaluatorNode::clustersCallback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&CostEvaluatorNode::odomCallback, this, std::placeholders::_1));
        
        // Publisher
        table_pub_ = create_publisher<frontier_interfaces::msg::FrontierTable>(
            "/frontier_table", 10);
        
        RCLCPP_INFO(get_logger(), "Cost Evaluator started (w_dist: %.2f, w_size: %.2f)",
            w_distance_, w_size_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_z_ = msg->pose.pose.position.z;
        pose_received_ = true;
    }
    
    void clustersCallback(const frontier_interfaces::msg::FrontierTable::SharedPtr msg) {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for odometry...");
            return;
        }
        
        double rx, ry, rz;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            rx = robot_x_; ry = robot_y_; rz = robot_z_;
        }
        
        // Copy and update costs
        auto table = *msg;
        table.header.stamp = now();
        
        uint32_t reachable = 0;
        
        for (auto& cluster : table.clusters) {
            // Distance to centroid
            double dx = cluster.centroid.x - rx;
            double dy = cluster.centroid.y - ry;
            double dz = cluster.centroid.z - rz;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Mark unreachable if too far
            if (distance > max_distance_) {
                cluster.reachable = false;
                cluster.cost = std::numeric_limits<double>::max();
                cluster.utility = 0.0;
                continue;
            }
            
            cluster.reachable = true;
            reachable++;
            
            // Cost function (lower is better)
            // Cost = w_distance * distance - w_size * log(size) + w_height * |z - preferred|
            double distance_cost = w_distance_ * distance;
            double size_benefit = w_size_ * std::log(cluster.size + 1);
            double height_penalty = w_height_ * std::abs(cluster.centroid.z - preferred_height_);
            
            cluster.cost = distance_cost - size_benefit + height_penalty;
            
            // Utility is inverse of cost (higher is better)
            cluster.utility = 1.0 / (cluster.cost + 0.1);
        }
        
        table.reachable_count = reachable;
        table_pub_->publish(table);
        
        RCLCPP_DEBUG(get_logger(), "Evaluated %zu clusters, %u reachable",
            table.clusters.size(), reachable);
    }
    
    // Cost weights
    double w_distance_;
    double w_size_;
    double w_height_;
    double preferred_height_;
    double max_distance_;
    
    // Robot pose
    double robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
    bool pose_received_ = false;
    std::mutex pose_mutex_;
    
    // ROS
    rclcpp::Subscription<frontier_interfaces::msg::FrontierTable>::SharedPtr clusters_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<frontier_interfaces::msg::FrontierTable>::SharedPtr table_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostEvaluatorNode>());
    rclcpp::shutdown();
    return 0;
}
