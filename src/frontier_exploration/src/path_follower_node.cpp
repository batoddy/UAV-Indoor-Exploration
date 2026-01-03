/**
 * Node 7: Path Follower
 * 
 * Input:  /local_path (nav_msgs/Path)
 *         /odom (nav_msgs/Odometry)
 * Output: /cmd_vel (geometry_msgs/Twist)
 * 
 * Local path'i takip eder, cmd_vel komutu üretir.
 * Pure Pursuit + altitude control.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower") {
        // Parameters
        declare_parameter("max_linear_velocity", 0.5);
        declare_parameter("max_angular_velocity", 0.5);
        declare_parameter("max_vertical_velocity", 0.3);
        declare_parameter("lookahead_distance", 1.5);
        declare_parameter("goal_tolerance", 0.5);
        declare_parameter("heading_tolerance", 0.3);  // Radians
        declare_parameter("control_rate", 20.0);
        
        max_linear_ = get_parameter("max_linear_velocity").as_double();
        max_angular_ = get_parameter("max_angular_velocity").as_double();
        max_vertical_ = get_parameter("max_vertical_velocity").as_double();
        lookahead_ = get_parameter("lookahead_distance").as_double();
        goal_tol_ = get_parameter("goal_tolerance").as_double();
        heading_tol_ = get_parameter("heading_tolerance").as_double();
        
        // Subscribers
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10,
            std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));
        
        // Publisher
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Control timer
        double rate = get_parameter("control_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&PathFollowerNode::controlLoop, this));
        
        RCLCPP_INFO(get_logger(), "Path Follower started (v_max: %.2f, w_max: %.2f)",
            max_linear_, max_angular_);
    }

private:
    struct Point3D {
        double x, y, z;
        double distance(const Point3D& o) const {
            double dx = x - o.x, dy = y - o.y, dz = z - o.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        double distance2D(const Point3D& o) const {
            double dx = x - o.x, dy = y - o.y;
            return std::sqrt(dx*dx + dy*dy);
        }
    };
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_.clear();
        for (const auto& ps : msg->poses) {
            path_.push_back({
                ps.pose.position.x,
                ps.pose.position.y,
                ps.pose.position.z
            });
        }
        has_path_ = !path_.empty();
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_.x = msg->pose.pose.position.x;
        robot_.y = msg->pose.pose.position.y;
        robot_.z = msg->pose.pose.position.z;
        
        // Extract yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        pose_received_ = true;
    }
    
    void controlLoop() {
        geometry_msgs::msg::Twist cmd;
        
        if (!has_path_ || !pose_received_) {
            cmd_pub_->publish(cmd);  // Zero velocity
            return;
        }
        
        Point3D robot;
        double yaw;
        std::vector<Point3D> path;
        {
            std::lock_guard<std::mutex> lock1(pose_mutex_);
            std::lock_guard<std::mutex> lock2(path_mutex_);
            robot = robot_;
            yaw = yaw_;
            path = path_;
        }
        
        if (path.empty()) {
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Find lookahead point
        Point3D target = findLookahead(robot, path);
        
        // Check if goal reached
        if (robot.distance(path.back()) < goal_tol_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Near goal, holding position");
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Compute control
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double dz = target.z - robot.z;
        
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalizeAngle(target_yaw - yaw);
        
        double horizontal_dist = std::sqrt(dx*dx + dy*dy);
        
        // Pure pursuit control
        if (std::abs(yaw_error) > heading_tol_) {
            // Rotate toward target first
            cmd.angular.z = std::clamp(yaw_error * 1.5, -max_angular_, max_angular_);
            cmd.linear.x = max_linear_ * 0.2;  // Slow forward while rotating
        } else {
            // Move forward with proportional steering
            cmd.linear.x = max_linear_;
            cmd.angular.z = std::clamp(yaw_error * 2.0, -max_angular_, max_angular_);
            
            // Slow down near goal
            double dist_to_goal = robot.distance(path.back());
            if (dist_to_goal < 2.0) {
                cmd.linear.x *= std::max(0.3, dist_to_goal / 2.0);
            }
        }
        
        // Altitude control
        cmd.linear.z = std::clamp(dz * 0.8, -max_vertical_, max_vertical_);
        
        cmd_pub_->publish(cmd);
    }
    
    Point3D findLookahead(const Point3D& robot, const std::vector<Point3D>& path) {
        // Find point on path that is lookahead distance away
        for (size_t i = 0; i < path.size(); ++i) {
            if (robot.distance(path[i]) >= lookahead_) {
                return path[i];
            }
        }
        return path.back();
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // Parameters
    double max_linear_;
    double max_angular_;
    double max_vertical_;
    double lookahead_;
    double goal_tol_;
    double heading_tol_;
    
    // State
    std::vector<Point3D> path_;
    bool has_path_ = false;
    std::mutex path_mutex_;
    
    Point3D robot_;
    double yaw_ = 0;
    bool pose_received_ = false;
    std::mutex pose_mutex_;
    
    // ROS
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
