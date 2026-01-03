/**
 * Node 4: Goal Selector
 * 
 * Input:  /frontier_table (frontier_interfaces/FrontierTable)
 * Output: /selected_goal (geometry_msgs/PoseStamped)
 *         /exploration_state (frontier_interfaces/ExplorationState)
 * 
 * Cost'a göre en iyi frontier'ı seçer.
 * Şimdilik tek bir goal seçiyor, sonra 1-2-3 şeklinde küme seçilebilir.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <frontier_interfaces/msg/frontier_table.hpp>
#include <frontier_interfaces/msg/exploration_state.hpp>

#include <set>

class GoalSelectorNode : public rclcpp::Node {
public:
    GoalSelectorNode() : Node("goal_selector") {
        // Parameters
        declare_parameter("selection_mode", "lowest_cost");  // lowest_cost, nearest, largest
        declare_parameter("goal_reached_threshold", 1.0);
        declare_parameter("replan_threshold", 5.0);  // Replan if better goal found by this margin
        declare_parameter("blacklist_timeout", 30.0);  // Seconds before retrying failed goals
        
        selection_mode_ = get_parameter("selection_mode").as_string();
        goal_threshold_ = get_parameter("goal_reached_threshold").as_double();
        replan_threshold_ = get_parameter("replan_threshold").as_double();
        
        // Subscribers
        table_sub_ = create_subscription<frontier_interfaces::msg::FrontierTable>(
            "/frontier_table", 10,
            std::bind(&GoalSelectorNode::tableCallback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GoalSelectorNode::odomCallback, this, std::placeholders::_1));
        
        // Publishers
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/selected_goal", 10);
        
        state_pub_ = create_publisher<frontier_interfaces::msg::ExplorationState>(
            "/exploration_state", 10);
        
        RCLCPP_INFO(get_logger(), "Goal Selector started (mode: %s)", selection_mode_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_z_ = msg->pose.pose.position.z;
        pose_received_ = true;
        
        // Check if current goal reached
        if (has_goal_) {
            double dx = current_goal_.x - robot_x_;
            double dy = current_goal_.y - robot_y_;
            double dz = current_goal_.z - robot_z_;
            distance_to_goal_ = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance_to_goal_ < goal_threshold_) {
                RCLCPP_INFO(get_logger(), "Goal %u reached!", current_goal_id_);
                visited_goals_.insert(current_goal_id_);
                has_goal_ = false;
                state_ = frontier_interfaces::msg::ExplorationState::GOAL_REACHED;
            }
        }
    }
    
    void tableCallback(const frontier_interfaces::msg::FrontierTable::SharedPtr msg) {
        if (!pose_received_) return;
        
        // Find best goal
        const frontier_interfaces::msg::FrontierCluster* best = nullptr;
        double best_cost = std::numeric_limits<double>::max();
        
        double rx, ry, rz;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            rx = robot_x_; ry = robot_y_; rz = robot_z_;
        }
        
        for (const auto& cluster : msg->clusters) {
            // Skip unreachable or visited
            if (!cluster.reachable || visited_goals_.count(cluster.id)) {
                continue;
            }
            
            double score;
            if (selection_mode_ == "nearest") {
                double dx = cluster.centroid.x - rx;
                double dy = cluster.centroid.y - ry;
                double dz = cluster.centroid.z - rz;
                score = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (selection_mode_ == "largest") {
                score = -static_cast<double>(cluster.size);  // Negative for min selection
            } else {  // lowest_cost
                score = cluster.cost;
            }
            
            if (score < best_cost) {
                best_cost = score;
                best = &cluster;
            }
        }
        
        // Update state
        auto state_msg = frontier_interfaces::msg::ExplorationState();
        state_msg.header.stamp = now();
        state_msg.header.frame_id = msg->header.frame_id;
        
        if (!best) {
            if (msg->clusters.empty()) {
                state_ = frontier_interfaces::msg::ExplorationState::COMPLETE;
                state_msg.status_message = "No frontiers - exploration complete!";
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Exploration complete!");
            } else {
                state_ = frontier_interfaces::msg::ExplorationState::STUCK;
                state_msg.status_message = "All frontiers visited or unreachable";
            }
            state_msg.state = state_;
            state_pub_->publish(state_msg);
            return;
        }
        
        // Check if we should switch goals
        bool should_update = false;
        if (!has_goal_) {
            should_update = true;
        } else if (best->id != current_goal_id_) {
            // Switch if new goal is significantly better
            double improvement = current_goal_cost_ - best->cost;
            if (improvement > replan_threshold_) {
                RCLCPP_INFO(get_logger(), "Switching to better goal %u (improvement: %.2f)",
                    best->id, improvement);
                should_update = true;
            }
        }
        
        if (should_update) {
            current_goal_id_ = best->id;
            current_goal_ = best->centroid;
            current_goal_cost_ = best->cost;
            has_goal_ = true;
            state_ = frontier_interfaces::msg::ExplorationState::PLANNING;
            
            // Publish goal
            auto goal_msg = geometry_msgs::msg::PoseStamped();
            goal_msg.header.stamp = now();
            goal_msg.header.frame_id = msg->header.frame_id;
            goal_msg.pose.position = best->centroid;
            goal_msg.pose.orientation.w = 1.0;
            
            goal_pub_->publish(goal_msg);
            
            RCLCPP_INFO(get_logger(), "Selected goal %u at (%.2f, %.2f, %.2f) cost: %.2f",
                best->id, best->centroid.x, best->centroid.y, best->centroid.z, best->cost);
        }
        
        // Publish state
        state_msg.state = state_;
        state_msg.current_goal_id = current_goal_id_;
        state_msg.current_goal = current_goal_;
        state_msg.distance_to_goal = distance_to_goal_;
        state_msg.status_message = "Navigating to frontier " + std::to_string(current_goal_id_);
        state_pub_->publish(state_msg);
    }
    
    // Parameters
    std::string selection_mode_;
    double goal_threshold_;
    double replan_threshold_;
    
    // State
    bool has_goal_ = false;
    uint32_t current_goal_id_ = 0;
    geometry_msgs::msg::Point current_goal_;
    double current_goal_cost_ = 0;
    double distance_to_goal_ = 0;
    uint8_t state_ = frontier_interfaces::msg::ExplorationState::IDLE;
    std::set<uint32_t> visited_goals_;
    
    // Robot pose
    double robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
    bool pose_received_ = false;
    std::mutex pose_mutex_;
    
    // ROS
    rclcpp::Subscription<frontier_interfaces::msg::FrontierTable>::SharedPtr table_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<frontier_interfaces::msg::ExplorationState>::SharedPtr state_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalSelectorNode>());
    rclcpp::shutdown();
    return 0;
}
