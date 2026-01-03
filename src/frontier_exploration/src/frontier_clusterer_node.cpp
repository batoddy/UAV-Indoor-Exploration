/**
 * Node 2: Frontier Clusterer
 * 
 * Input:  /frontier_points_batch (sensor_msgs/PointCloud2)
 * Output: /frontier_clusters (frontier_interfaces/FrontierTable)
 * 
 * Raw frontier noktalarını cluster'lar, her cluster için centroid hesaplar.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <frontier_interfaces/msg/frontier_table.hpp>
#include <frontier_interfaces/msg/frontier_cluster.hpp>

#include <queue>
#include <unordered_set>

class FrontierClustererNode : public rclcpp::Node {
public:
    FrontierClustererNode() : Node("frontier_clusterer"), next_cluster_id_(1) {
        // Parameters
        declare_parameter("cluster_tolerance", 0.6);
        declare_parameter("min_cluster_size", 5);
        declare_parameter("max_cluster_size", 10000);
        
        cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = get_parameter("max_cluster_size").as_int();
        
        // Subscriber
        points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/frontier_points_batch", 10,
            std::bind(&FrontierClustererNode::pointsCallback, this, std::placeholders::_1));
        
        // Publisher
        table_pub_ = create_publisher<frontier_interfaces::msg::FrontierTable>(
            "/frontier_clusters", 10);
        
        RCLCPP_INFO(get_logger(), "Frontier Clusterer started (tol: %.2f, min: %d)",
            cluster_tolerance_, min_cluster_size_);
    }

private:
    struct Point3D {
        double x, y, z;
        
        double distanceSq(const Point3D& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            double dz = z - other.z;
            return dx*dx + dy*dy + dz*dz;
        }
    };
    
    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Extract points from PointCloud2
        std::vector<Point3D> points;
        points.reserve(msg->width);
        
        for (size_t i = 0; i < msg->width; ++i) {
            const float* ptr = reinterpret_cast<const float*>(&msg->data[i * msg->point_step]);
            points.push_back({ptr[0], ptr[1], ptr[2]});
        }
        
        if (points.empty()) {
            // Publish empty table
            auto table = frontier_interfaces::msg::FrontierTable();
            table.header.stamp = now();
            table.header.frame_id = msg->header.frame_id;
            table.total_frontiers = 0;
            table_pub_->publish(table);
            return;
        }
        
        // Cluster points using simple region growing
        auto clusters = clusterPoints(points);
        
        // Build FrontierTable message
        auto table = frontier_interfaces::msg::FrontierTable();
        table.header.stamp = now();
        table.header.frame_id = msg->header.frame_id;
        
        for (auto& cluster_points : clusters) {
            if (static_cast<int>(cluster_points.size()) < min_cluster_size_ ||
                static_cast<int>(cluster_points.size()) > max_cluster_size_) {
                continue;
            }
            
            frontier_interfaces::msg::FrontierCluster fc;
            fc.id = next_cluster_id_++;
            fc.size = cluster_points.size();
            fc.visited = false;
            fc.reachable = true;  // Will be updated by cost evaluator
            fc.cost = 0.0;
            fc.utility = 0.0;
            
            // Compute centroid
            double cx = 0, cy = 0, cz = 0;
            for (const auto& p : cluster_points) {
                geometry_msgs::msg::Point gp;
                gp.x = p.x; gp.y = p.y; gp.z = p.z;
                fc.points.push_back(gp);
                cx += p.x; cy += p.y; cz += p.z;
            }
            fc.centroid.x = cx / cluster_points.size();
            fc.centroid.y = cy / cluster_points.size();
            fc.centroid.z = cz / cluster_points.size();
            
            table.clusters.push_back(fc);
        }
        
        table.total_frontiers = table.clusters.size();
        table.reachable_count = table.clusters.size();
        table.selected_id = 0;
        
        table_pub_->publish(table);
        
        RCLCPP_DEBUG(get_logger(), "Clustered %zu points into %zu clusters",
            points.size(), table.clusters.size());
    }
    
    std::vector<std::vector<Point3D>> clusterPoints(const std::vector<Point3D>& points) {
        std::vector<std::vector<Point3D>> clusters;
        std::vector<bool> visited(points.size(), false);
        
        double tol_sq = cluster_tolerance_ * cluster_tolerance_;
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;
            
            std::vector<Point3D> cluster;
            std::queue<size_t> queue;
            queue.push(i);
            visited[i] = true;
            
            while (!queue.empty()) {
                size_t idx = queue.front();
                queue.pop();
                cluster.push_back(points[idx]);
                
                // Find neighbors
                for (size_t j = 0; j < points.size(); ++j) {
                    if (!visited[j] && points[idx].distanceSq(points[j]) <= tol_sq) {
                        visited[j] = true;
                        queue.push(j);
                    }
                }
            }
            
            clusters.push_back(std::move(cluster));
        }
        
        return clusters;
    }
    
    // State
    uint32_t next_cluster_id_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Publisher<frontier_interfaces::msg::FrontierTable>::SharedPtr table_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierClustererNode>());
    rclcpp::shutdown();
    return 0;
}
