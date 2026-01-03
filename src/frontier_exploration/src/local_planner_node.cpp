/**
 * Node 6: Local Planner
 * 
 * Input:  /global_path (nav_msgs/Path)
 *         /octomap_binary (octomap_msgs/Octomap)
 *         /odom (nav_msgs/Odometry)
 * Output: /local_path (nav_msgs/Path)
 * 
 * Global path'i alır, real-time obstacle avoidance yapar.
 * - DWA-style velocity sampling
 * - Dynamic obstacle avoidance
 * - Path smoothing
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

using namespace std::chrono_literals;

class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode() : Node("local_planner") {
        // Parameters
        declare_parameter("local_horizon", 5.0);       // Meters to look ahead
        declare_parameter("safety_margin", 0.4);
        declare_parameter("replan_rate", 10.0);        // Hz
        declare_parameter("carrot_distance", 2.0);     // Lookahead distance
        declare_parameter("obstacle_check_radius", 3.0);
        
        local_horizon_ = get_parameter("local_horizon").as_double();
        safety_margin_ = get_parameter("safety_margin").as_double();
        carrot_dist_ = get_parameter("carrot_distance").as_double();
        obstacle_radius_ = get_parameter("obstacle_check_radius").as_double();
        
        // Subscribers
        global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10,
            std::bind(&LocalPlannerNode::globalPathCallback, this, std::placeholders::_1));
        
        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&LocalPlannerNode::octomapCallback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));
        
        // Publisher
        local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);
        
        // Timer for local planning
        double rate = get_parameter("replan_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&LocalPlannerNode::planLocal, this));
        
        RCLCPP_INFO(get_logger(), "Local Planner started");
    }

private:
    struct Point3D {
        double x, y, z;
        double distance(const Point3D& o) const {
            double dx = x - o.x, dy = y - o.y, dz = z - o.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    };
    
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        auto* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_pos_.x = msg->pose.pose.position.x;
        robot_pos_.y = msg->pose.pose.position.y;
        robot_pos_.z = msg->pose.pose.position.z;
        
        // Extract yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        pose_received_ = true;
    }
    
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        global_path_.clear();
        for (const auto& ps : msg->poses) {
            global_path_.push_back({
                ps.pose.position.x,
                ps.pose.position.y,
                ps.pose.position.z
            });
        }
        frame_id_ = msg->header.frame_id;
        has_path_ = !global_path_.empty();
        
        RCLCPP_DEBUG(get_logger(), "Received global path with %zu waypoints", global_path_.size());
    }
    
    void planLocal() {
        if (!has_path_ || !pose_received_ || !octree_) return;
        
        Point3D robot;
        double yaw;
        std::vector<Point3D> global;
        {
            std::lock_guard<std::mutex> lock1(pose_mutex_);
            std::lock_guard<std::mutex> lock2(path_mutex_);
            robot = robot_pos_;
            yaw = robot_yaw_;
            global = global_path_;
        }
        
        if (global.empty()) return;
        
        // Find closest point on path
        size_t closest_idx = 0;
        double closest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global.size(); ++i) {
            double d = robot.distance(global[i]);
            if (d < closest_dist) {
                closest_dist = d;
                closest_idx = i;
            }
        }
        
        // Find carrot point (lookahead)
        Point3D carrot = global.back();
        double accum_dist = 0;
        for (size_t i = closest_idx; i < global.size() - 1; ++i) {
            accum_dist += global[i].distance(global[i + 1]);
            if (accum_dist >= carrot_dist_) {
                carrot = global[i + 1];
                break;
            }
        }
        
        // Check if direct path to carrot is clear
        std::vector<Point3D> local_path;
        local_path.push_back(robot);
        
        if (isPathClear(robot, carrot)) {
            // Direct path is clear
            local_path.push_back(carrot);
        } else {
            // Need to find alternative - simple potential field avoidance
            Point3D adjusted = findClearPoint(robot, carrot, yaw);
            if (adjusted.distance(robot) > 0.1) {
                local_path.push_back(adjusted);
            }
            local_path.push_back(carrot);
        }
        
        // Add remaining global path within horizon
        for (size_t i = closest_idx + 1; i < global.size(); ++i) {
            if (robot.distance(global[i]) > local_horizon_) break;
            local_path.push_back(global[i]);
        }
        
        // Publish local path
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = now();
        path_msg.header.frame_id = frame_id_;
        
        for (const auto& p : local_path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.position.z = p.z;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }
        
        local_path_pub_->publish(path_msg);
    }
    
    bool isPathClear(const Point3D& a, const Point3D& b) {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        if (!octree_) return true;
        
        double dist = a.distance(b);
        int steps = static_cast<int>(dist / 0.2) + 1;
        
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            Point3D p = {
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z)
            };
            
            if (!isPointClear(p)) return false;
        }
        return true;
    }
    
    bool isPointClear(const Point3D& p) {
        // Check safety sphere
        double res = octree_->getResolution();
        int checks = static_cast<int>(safety_margin_ / res);
        
        for (int dx = -checks; dx <= checks; ++dx) {
            for (int dy = -checks; dy <= checks; ++dy) {
                for (int dz = -checks; dz <= checks; ++dz) {
                    double d = std::sqrt(dx*dx + dy*dy + dz*dz) * res;
                    if (d > safety_margin_) continue;
                    
                    octomap::point3d q(p.x + dx*res, p.y + dy*res, p.z + dz*res);
                    auto* node = octree_->search(q);
                    if (node && octree_->isNodeOccupied(node)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    Point3D findClearPoint(const Point3D& robot, const Point3D& goal, double yaw) {
        // Simple: sample points around robot and pick best clear one toward goal
        std::lock_guard<std::mutex> lock(octree_mutex_);
        
        double best_score = -std::numeric_limits<double>::max();
        Point3D best = robot;
        
        // Sample in a hemisphere toward goal
        double target_yaw = std::atan2(goal.y - robot.y, goal.x - robot.x);
        
        for (double angle = -M_PI/2; angle <= M_PI/2; angle += M_PI/8) {
            for (double dist = 0.5; dist <= 2.0; dist += 0.5) {
                for (double dz = -0.5; dz <= 0.5; dz += 0.5) {
                    double actual_yaw = target_yaw + angle;
                    Point3D candidate = {
                        robot.x + dist * std::cos(actual_yaw),
                        robot.y + dist * std::sin(actual_yaw),
                        robot.z + dz
                    };
                    
                    if (!isPointClear(candidate)) continue;
                    
                    // Score: closer to goal direction, farther from robot
                    double progress = -candidate.distance(goal);  // Negative = closer is better
                    double deviation = std::abs(angle);
                    double score = progress - deviation * 2.0;
                    
                    if (score > best_score) {
                        best_score = score;
                        best = candidate;
                    }
                }
            }
        }
        
        return best;
    }
    
    // Parameters
    double local_horizon_;
    double safety_margin_;
    double carrot_dist_;
    double obstacle_radius_;
    
    // State
    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex octree_mutex_;
    
    Point3D robot_pos_;
    double robot_yaw_ = 0;
    bool pose_received_ = false;
    std::mutex pose_mutex_;
    
    std::vector<Point3D> global_path_;
    std::string frame_id_;
    bool has_path_ = false;
    std::mutex path_mutex_;
    
    // ROS
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
