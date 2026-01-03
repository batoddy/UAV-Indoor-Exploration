/**
 * Node 5: Global Planner (3D A*)
 * 
 * Input:  /selected_goal (geometry_msgs/PoseStamped)
 *         /octomap_binary (octomap_msgs/Octomap)
 *         /odom (nav_msgs/Odometry)
 * Output: /global_path (nav_msgs/Path)
 * 
 * Seçilen hedefe 3D A* ile collision-free path hesaplar.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <queue>
#include <unordered_set>
#include <unordered_map>

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner") {
        // Parameters
        declare_parameter("planning_resolution", 0.3);
        declare_parameter("safety_margin", 0.5);
        declare_parameter("max_iterations", 100000);
        declare_parameter("allow_unknown", true);
        
        resolution_ = get_parameter("planning_resolution").as_double();
        safety_margin_ = get_parameter("safety_margin").as_double();
        max_iterations_ = get_parameter("max_iterations").as_int();
        allow_unknown_ = get_parameter("allow_unknown").as_bool();
        
        // Subscribers
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/selected_goal", 10,
            std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));
        
        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&GlobalPlannerNode::octomapCallback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GlobalPlannerNode::odomCallback, this, std::placeholders::_1));
        
        // Publisher
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        
        RCLCPP_INFO(get_logger(), "Global Planner started (res: %.2f, margin: %.2f)",
            resolution_, safety_margin_);
    }

private:
    struct Point3D {
        double x, y, z;
        double distance(const Point3D& o) const {
            double dx = x - o.x, dy = y - o.y, dz = z - o.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    };
    
    struct AStarNode {
        Point3D pos;
        double g, h;
        double f() const { return g + h; }
        std::shared_ptr<AStarNode> parent;
    };
    
    struct NodeCmp {
        bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
            return a->f() > b->f();
        }
    };
    
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        auto* tree = octomap_msgs::msgToMap(*msg);
        if (tree) octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        start_.x = msg->pose.pose.position.x;
        start_.y = msg->pose.pose.position.y;
        start_.z = msg->pose.pose.position.z;
        pose_received_ = true;
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!pose_received_ || !octree_) {
            RCLCPP_WARN(get_logger(), "Cannot plan: waiting for odom/octomap");
            return;
        }
        
        Point3D goal = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
        Point3D start;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            start = start_;
        }
        
        RCLCPP_INFO(get_logger(), "Planning from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",
            start.x, start.y, start.z, goal.x, goal.y, goal.z);
        
        auto path = planPath(start, goal);
        
        if (path.empty()) {
            RCLCPP_WARN(get_logger(), "No path found!");
            return;
        }
        
        // Smooth path
        path = smoothPath(path);
        
        // Publish
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = now();
        path_msg.header.frame_id = msg->header.frame_id;
        
        for (const auto& p : path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.position.z = p.z;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }
        
        path_pub_->publish(path_msg);
        RCLCPP_INFO(get_logger(), "Published path with %zu waypoints", path.size());
    }
    
    std::string toKey(const Point3D& p) const {
        int x = static_cast<int>(std::round(p.x / resolution_));
        int y = static_cast<int>(std::round(p.y / resolution_));
        int z = static_cast<int>(std::round(p.z / resolution_));
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
    }
    
    bool isCollisionFree(const Point3D& p) {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        if (!octree_) return false;
        
        // Check point and safety sphere
        int checks = static_cast<int>(safety_margin_ / octree_->getResolution());
        double res = octree_->getResolution();
        
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
                    if (!node && !allow_unknown_) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    std::vector<Point3D> getNeighbors(const Point3D& p) {
        std::vector<Point3D> neighbors;
        // 26-connected
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    neighbors.push_back({
                        p.x + dx * resolution_,
                        p.y + dy * resolution_,
                        p.z + dz * resolution_
                    });
                }
            }
        }
        return neighbors;
    }
    
    std::vector<Point3D> planPath(const Point3D& start, const Point3D& goal) {
        std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, NodeCmp> open;
        std::unordered_set<std::string> closed;
        std::unordered_map<std::string, std::shared_ptr<AStarNode>> open_map;
        
        auto start_node = std::make_shared<AStarNode>();
        start_node->pos = start;
        start_node->g = 0;
        start_node->h = start.distance(goal);
        start_node->parent = nullptr;
        
        open.push(start_node);
        open_map[toKey(start)] = start_node;
        
        int iter = 0;
        while (!open.empty() && iter < max_iterations_) {
            ++iter;
            
            auto current = open.top();
            open.pop();
            
            std::string key = toKey(current->pos);
            if (closed.count(key)) continue;
            closed.insert(key);
            
            // Goal check
            if (current->pos.distance(goal) < resolution_ * 2) {
                // Reconstruct path
                std::vector<Point3D> path;
                auto n = current;
                while (n) {
                    path.push_back(n->pos);
                    n = n->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Expand
            for (const auto& np : getNeighbors(current->pos)) {
                std::string nkey = toKey(np);
                if (closed.count(nkey)) continue;
                if (!isCollisionFree(np)) continue;
                
                double new_g = current->g + current->pos.distance(np);
                
                if (open_map.count(nkey) && open_map[nkey]->g <= new_g) continue;
                
                auto nn = std::make_shared<AStarNode>();
                nn->pos = np;
                nn->g = new_g;
                nn->h = np.distance(goal);
                nn->parent = current;
                
                open.push(nn);
                open_map[nkey] = nn;
            }
        }
        
        RCLCPP_WARN(get_logger(), "A* failed after %d iterations", iter);
        return {};
    }
    
    std::vector<Point3D> smoothPath(const std::vector<Point3D>& path) {
        if (path.size() < 3) return path;
        
        std::vector<Point3D> smooth;
        smooth.push_back(path[0]);
        
        size_t i = 0;
        while (i < path.size() - 1) {
            // Try to skip waypoints
            size_t j = path.size() - 1;
            while (j > i + 1) {
                if (isLineClear(path[i], path[j])) break;
                --j;
            }
            smooth.push_back(path[j]);
            i = j;
        }
        
        return smooth;
    }
    
    bool isLineClear(const Point3D& a, const Point3D& b) {
        double dist = a.distance(b);
        int steps = static_cast<int>(dist / (resolution_ * 0.5)) + 1;
        
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            Point3D p = {
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z)
            };
            if (!isCollisionFree(p)) return false;
        }
        return true;
    }
    
    // Parameters
    double resolution_;
    double safety_margin_;
    int max_iterations_;
    bool allow_unknown_;
    
    // State
    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex octree_mutex_;
    Point3D start_;
    bool pose_received_ = false;
    std::mutex pose_mutex_;
    
    // ROS
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}