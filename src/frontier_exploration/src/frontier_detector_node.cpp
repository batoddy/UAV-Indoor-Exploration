/**
 * Node 1: Frontier Detector
 * 
 * Input:  /octomap_binary (octomap_msgs/Octomap)
 * Output: /frontier_points (frontier_interfaces/FrontierPoint[])
 * 
 * OctoMap'i tarar, FREE hücreler içinde UNKNOWN komşusu olanları frontier olarak işaretler.
 */

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

class FrontierDetectorNode : public rclcpp::Node {
public:
    FrontierDetectorNode() : Node("frontier_detector") {
        // Parameters
        declare_parameter("min_z", 0.5);
        declare_parameter("max_z", 8.0);
        declare_parameter("detection_rate", 2.0);
        
        min_z_ = get_parameter("min_z").as_double();
        max_z_ = get_parameter("max_z").as_double();
        double rate = get_parameter("detection_rate").as_double();
        
        // Subscriber
        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&FrontierDetectorNode::octomapCallback, this, std::placeholders::_1));
        
        // Publisher - array of frontier points
        frontier_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/frontier_points", 10);
        
        // Batch publisher for efficiency
        frontier_batch_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/frontier_points_batch", 10);
        
        // Timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&FrontierDetectorNode::detectFrontiers, this));
        
        RCLCPP_INFO(get_logger(), "Frontier Detector started (z: %.1f - %.1f)", min_z_, max_z_);
    }

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
        }
    }
    
    bool isFrontierCell(const octomap::point3d& point) {
        if (!octree_) return false;
        
        octomap::OcTreeNode* node = octree_->search(point);
        
        // Must be known and free
        if (!node || octree_->isNodeOccupied(node)) {
            return false;
        }
        
        // Check 6-connected neighbors for unknown
        float res = static_cast<float>(octree_->getResolution());
        const std::array<octomap::point3d, 6> neighbors = {{
            octomap::point3d(point.x() + res, point.y(), point.z()),
            octomap::point3d(point.x() - res, point.y(), point.z()),
            octomap::point3d(point.x(), point.y() + res, point.z()),
            octomap::point3d(point.x(), point.y() - res, point.z()),
            octomap::point3d(point.x(), point.y(), point.z() + res),
            octomap::point3d(point.x(), point.y(), point.z() - res)
        }};
        
        for (const auto& n : neighbors) {
            if (!octree_->search(n)) {
                return true;  // Unknown neighbor found
            }
        }
        
        return false;
    }
    
    void detectFrontiers() {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        
        if (!octree_) return;
        
        std::vector<geometry_msgs::msg::Point> frontier_points;
        
        // Iterate through all leaf nodes
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            octomap::point3d coord = it.getCoordinate();
            
            // Height filter
            if (coord.z() < min_z_ || coord.z() > max_z_) {
                continue;
            }
            
            if (isFrontierCell(coord)) {
                geometry_msgs::msg::Point p;
                p.x = coord.x();
                p.y = coord.y();
                p.z = coord.z();
                frontier_points.push_back(p);
            }
        }
        
        // Publish as PointCloud2 for efficiency
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.stamp = now();
        cloud_msg->header.frame_id = "odom";
        cloud_msg->height = 1;
        cloud_msg->width = frontier_points.size();
        cloud_msg->is_dense = true;
        cloud_msg->is_bigendian = false;
        
        // Define fields
        sensor_msgs::msg::PointField field_x, field_y, field_z;
        field_x.name = "x"; field_x.offset = 0; field_x.datatype = 7; field_x.count = 1;
        field_y.name = "y"; field_y.offset = 4; field_y.datatype = 7; field_y.count = 1;
        field_z.name = "z"; field_z.offset = 8; field_z.datatype = 7; field_z.count = 1;
        cloud_msg->fields = {field_x, field_y, field_z};
        cloud_msg->point_step = 12;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        
        // Fill data
        cloud_msg->data.resize(cloud_msg->row_step);
        for (size_t i = 0; i < frontier_points.size(); ++i) {
            float* ptr = reinterpret_cast<float*>(&cloud_msg->data[i * 12]);
            ptr[0] = static_cast<float>(frontier_points[i].x);
            ptr[1] = static_cast<float>(frontier_points[i].y);
            ptr[2] = static_cast<float>(frontier_points[i].z);
        }
        
        frontier_batch_pub_->publish(std::move(cloud_msg));
        
        RCLCPP_DEBUG(get_logger(), "Detected %zu frontier points", frontier_points.size());
    }
    
    // State
    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex octree_mutex_;
    double min_z_, max_z_;
    
    // ROS
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr frontier_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_batch_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDetectorNode>());
    rclcpp::shutdown();
    return 0;
}