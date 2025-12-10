#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Nav2ToPX4Bridge : public rclcpp::Node
{
public:
    Nav2ToPX4Bridge() : Node("nav2_to_px4_bridge")
    {
        // Parameters
        this->declare_parameter<double>("flight_altitude", 2.0);
        this->declare_parameter<double>("velocity_scale", 1.0);
        this->declare_parameter<double>("cmd_vel_timeout", 0.5);
        this->declare_parameter<bool>("auto_arm", true);
        
        flight_altitude_ = this->get_parameter("flight_altitude").as_double();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        auto_arm_ = this->get_parameter("auto_arm").as_bool();
        
        // QoS for PX4
        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();
        
        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Nav2ToPX4Bridge::cmdVelCallback, this, _1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&Nav2ToPX4Bridge::odomCallback, this, _1));
            
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4_qos,
            std::bind(&Nav2ToPX4Bridge::vehicleStatusCallback, this, _1));
        
        // Publishers
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
            
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
            
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        
        // Control loop timer - 50Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&Nav2ToPX4Bridge::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Nav2 to PX4 Bridge started");
        RCLCPP_INFO(this->get_logger(), "Flight altitude: %.2f m", flight_altitude_);
        RCLCPP_INFO(this->get_logger(), "Velocity scale: %.2f", velocity_scale_);
    }

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    geometry_msgs::msg::Twist current_cmd_vel_;
    nav_msgs::msg::Odometry current_odom_;
    rclcpp::Time last_cmd_vel_time_;
    bool cmd_vel_received_ = false;
    bool odom_received_ = false;
    bool is_armed_ = false;
    bool is_offboard_ = false;
    int offboard_setpoint_counter_ = 0;
    
    // Parameters
    double flight_altitude_;
    double velocity_scale_;
    double cmd_vel_timeout_;
    bool auto_arm_;
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_cmd_vel_ = *msg;
        last_cmd_vel_time_ = this->now();
        cmd_vel_received_ = true;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_received_ = true;
    }
    
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        is_armed_ = (msg->arming_state == 2);  // ARMED
        is_offboard_ = (msg->nav_state == 14); // OFFBOARD
    }
    
    void controlLoop()
    {
        if (!odom_received_) {
            return;
        }
        
        // Always publish offboard control mode
        publishOffboardControlMode();
        
        // Check if cmd_vel is fresh
        bool cmd_vel_fresh = false;
        if (cmd_vel_received_) {
            double dt = (this->now() - last_cmd_vel_time_).seconds();
            cmd_vel_fresh = (dt < cmd_vel_timeout_);
        }
        
        // Arm and engage offboard after enough setpoints
        if (offboard_setpoint_counter_ < 15) {
            offboard_setpoint_counter_++;
            
            // Hover at current position during startup
            publishHoverSetpoint();
            
            if (offboard_setpoint_counter_ == 10 && auto_arm_) {
                engageOffboardMode();
                arm();
            }
            return;
        }
        
        if (cmd_vel_fresh) {
            publishVelocitySetpoint();
        } else {
            // No fresh cmd_vel, hover in place
            publishHoverSetpoint();
        }
    }
    
    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = true;  // Velocity control mode
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }
    
    void publishVelocitySetpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        
        // Get current yaw from odometry
        double current_yaw = getYawFromQuaternion(current_odom_.pose.pose.orientation);
        
        // cmd_vel is in robot frame (base_link), convert to world frame (ENU)
        // Then convert ENU to NED for PX4
        double vx_body = current_cmd_vel_.linear.x * velocity_scale_;
        double vy_body = current_cmd_vel_.linear.y * velocity_scale_;
        
        // Rotate body velocities to world frame (ENU)
        double vx_enu = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
        double vy_enu = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
        
        // ENU to NED conversion
        // NED_x = ENU_y (North = East rotated)
        // NED_y = ENU_x (East = North rotated)  
        // NED_z = -ENU_z (Down = -Up)
        msg.velocity[0] = vx_enu;   // NED North
        msg.velocity[1] = -vy_enu;   // NED East
        msg.velocity[2] = 0.0f;     // NED Down (maintain altitude)
        
        // Yaw rate: ENU to NED (just negate)
        msg.yawspeed = -current_cmd_vel_.angular.z;
        
        // NaN for position means velocity control
        msg.position[0] = std::nanf("");
        msg.position[1] = std::nanf("");
        msg.position[2] = std::nanf("");
        
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }
    
    void publishHoverSetpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        
        // Zero velocity = hover
        msg.velocity[0] = 0.0f;
        msg.velocity[1] = 0.0f;
        msg.velocity[2] = 0.0f;
        msg.yawspeed = 0.0f;
        
        msg.position[0] = std::nanf("");
        msg.position[1] = std::nanf("");
        msg.position[2] = std::nanf("");
        
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }
    
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    void arm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }
    
    void disarm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    }
    
    void engageOffboardMode()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0, 6.0);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }
    
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2ToPX4Bridge>());
    rclcpp::shutdown();
    return 0;
}