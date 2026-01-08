/**
 * cmd_vel_px4_bridge - Offboard Control with Keep-Alive
 * 
 * IMPORTANT: PX4 requires continuous setpoints in offboard mode.
 * If no setpoint for 500ms, PX4 triggers failsafe (land/RTL).
 * 
 * This node ensures setpoints are ALWAYS sent, even without /cmd_vel.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <cmath>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("cmd_vel_px4_bridge")
    {
        declare_parameter<double>("takeoff_altitude", 1.5);
        declare_parameter<double>("cmd_timeout", 0.5);        // Timeout for cmd_vel
        declare_parameter<double>("hover_timeout", 30.0);     // Max hover time before warning
        
        takeoff_altitude_ = get_parameter("takeoff_altitude").as_double();
        cmd_timeout_ = get_parameter("cmd_timeout").as_double();
        hover_timeout_ = get_parameter("hover_timeout").as_double();

        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();

        // Subscribers
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                velocity_ = *msg;
                last_cmd_time_ = now();
                has_cmd_ = true;
                hover_start_time_.reset();  // Reset hover timer
            });

        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4_qos,
            [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                nav_state_ = msg->nav_state;
                arm_state_ = msg->arming_state;
                flight_check_ = msg->pre_flight_checks_pass;
                failsafe_ = msg->failsafe;
                
                // Failsafe algılandığında uyar
                if (failsafe_ && !last_failsafe_) {
                    RCLCPP_ERROR(get_logger(), "FAILSAFE TRIGGERED! nav_state=%d", nav_state_);
                }
                last_failsafe_ = failsafe_;
            });

        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", px4_qos,
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                pos_x_ = msg->x;
                pos_y_ = msg->y;
                pos_z_ = msg->z;
                yaw_ = msg->heading;
                has_pos_ = true;
            });

        // Publishers
        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Timers
        state_timer_ = create_wall_timer(100ms, [this]() { stateMachine(); });
        cmd_timer_ = create_wall_timer(20ms, [this]() { cmdLoop(); });  // 50Hz - CRITICAL!

        RCLCPP_INFO(get_logger(), "Offboard Control started (with keep-alive)");
        RCLCPP_INFO(get_logger(), "  Takeoff altitude: %.1f m", takeoff_altitude_);
        RCLCPP_INFO(get_logger(), "  Cmd timeout: %.1f s", cmd_timeout_);
    }

private:
    enum State { IDLE, ARMING, TAKEOFF_PREP, TAKEOFF, OFFBOARD };
    State state_ = IDLE;
    int counter_ = 0;

    void stateMachine()
    {
        switch (state_) {
            case IDLE:
                if (flight_check_) {
                    state_ = ARMING;
                    counter_ = 0;
                    RCLCPP_INFO(get_logger(), "State: ARMING");
                }
                break;

            case ARMING:
                if (!flight_check_) {
                    state_ = IDLE;
                } else if (arm_state_ == 2 && counter_ > 10) {
                    state_ = TAKEOFF_PREP;
                    counter_ = 0;
                    RCLCPP_INFO(get_logger(), "State: TAKEOFF_PREP");
                }
                arm();
                counter_++;
                break;

            case TAKEOFF_PREP:
                publishOffboardMode(true);
                publishPositionSetpoint(-takeoff_altitude_);
                
                if (++counter_ >= 20) {
                    setOffboardMode();
                    state_ = TAKEOFF;
                    counter_ = 0;
                    RCLCPP_INFO(get_logger(), "State: TAKEOFF to %.1fm", takeoff_altitude_);
                }
                break;

            case TAKEOFF:
                publishOffboardMode(true);
                publishPositionSetpoint(-takeoff_altitude_);
                
                if (-pos_z_ >= takeoff_altitude_ * 0.9) {
                    state_ = OFFBOARD;
                    offboard_mode_ = true;
                    // Hover pozisyonunu kaydet
                    hover_x_ = pos_x_;
                    hover_y_ = pos_y_;
                    hover_z_ = pos_z_;
                    RCLCPP_INFO(get_logger(), "State: OFFBOARD - Ready for cmd_vel!");
                }
                break;

            case OFFBOARD:
                if (!flight_check_ || arm_state_ != 2) {
                    state_ = IDLE;
                    offboard_mode_ = false;
                    RCLCPP_WARN(get_logger(), "Back to IDLE (disarmed or check failed)");
                }
                break;
        }
    }

    void cmdLoop()
    {
        if (!offboard_mode_ || !has_pos_) return;

        // ═══════════════════════════════════════════════════════════════
        // CRITICAL: Always publish offboard mode to keep PX4 happy
        // ═══════════════════════════════════════════════════════════════
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.timestamp = timestamp();
        
        // Check if we have recent cmd_vel
        double time_since_cmd = has_cmd_ ? (now() - last_cmd_time_).seconds() : 999.0;
        bool cmd_active = time_since_cmd < cmd_timeout_;

        px4_msgs::msg::TrajectorySetpoint traj_msg;
        traj_msg.timestamp = timestamp();

        if (cmd_active) {
            // ═══════════════════════════════════════════════════════════
            // VELOCITY CONTROL: Use cmd_vel
            // ═══════════════════════════════════════════════════════════
            mode_msg.position = false;
            mode_msg.velocity = true;
            
            // Body -> World frame transformation
            // ROS: X=forward, Y=left, Z=up
            // PX4 NED: X=north, Y=east, Z=down
            float vx = velocity_.linear.x;
            float vy = -velocity_.linear.y;  // ROS left -> PX4 right (east)
            float cos_yaw = std::cos(yaw_);
            float sin_yaw = std::sin(yaw_);

            traj_msg.velocity[0] = vx * cos_yaw - vy * sin_yaw;
            traj_msg.velocity[1] = vx * sin_yaw + vy * cos_yaw;
            traj_msg.velocity[2] = -velocity_.linear.z;  // ROS up -> PX4 down
            
            // Yaw rate: ROS CCW positive -> PX4 NED CW positive
            // In NED, positive yaw is clockwise when viewed from above
            traj_msg.yawspeed = -velocity_.angular.z;
            
            traj_msg.position = {NAN, NAN, NAN};
            traj_msg.yaw = NAN;
            
            // Debug: Log commanded velocities
            RCLCPP_DEBUG(get_logger(), 
                "cmd_vel: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f -> PX4: [%.2f, %.2f, %.2f] yawspeed=%.2f",
                velocity_.linear.x, velocity_.linear.y, velocity_.linear.z, velocity_.angular.z,
                traj_msg.velocity[0], traj_msg.velocity[1], traj_msg.velocity[2], traj_msg.yawspeed);
            
        } else {
            // ═══════════════════════════════════════════════════════════
            // POSITION HOLD: No cmd_vel, hold current position
            // ═══════════════════════════════════════════════════════════
            mode_msg.position = true;
            mode_msg.velocity = false;
            
            // İlk hover'da pozisyonu kaydet
            if (!hover_start_time_.has_value()) {
                hover_start_time_ = now();
                hover_x_ = pos_x_;
                hover_y_ = pos_y_;
                hover_z_ = pos_z_;
                RCLCPP_WARN(get_logger(), "No cmd_vel - holding position at (%.1f, %.1f, %.1f)",
                            hover_x_, hover_y_, -hover_z_);
            }
            
            // Hover timeout kontrolü
            double hover_duration = (now() - *hover_start_time_).seconds();
            if (hover_duration > hover_timeout_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                    "Hovering for %.0f seconds without cmd_vel!", hover_duration);
            }
            
            // Position hold setpoint
            traj_msg.position[0] = hover_x_;
            traj_msg.position[1] = hover_y_;
            traj_msg.position[2] = hover_z_;
            traj_msg.velocity = {NAN, NAN, NAN};
            traj_msg.yaw = NAN;
            traj_msg.yawspeed = NAN;
        }

        // ═══════════════════════════════════════════════════════════════
        // ALWAYS PUBLISH - This keeps offboard mode alive!
        // ═══════════════════════════════════════════════════════════════
        offboard_pub_->publish(mode_msg);
        trajectory_pub_->publish(traj_msg);
    }

    void publishOffboardMode(bool position)
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = timestamp();
        msg.position = position;
        msg.velocity = !position;
        offboard_pub_->publish(msg);
    }

    void publishPositionSetpoint(float z)
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = timestamp();
        msg.position = {NAN, NAN, z};
        msg.yaw = NAN;
        trajectory_pub_->publish(msg);
    }

    void arm()
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = timestamp();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
    }

    void setOffboardMode()
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = timestamp();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0;
        msg.param2 = 6.0;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
    }

    uint64_t timestamp() { return get_clock()->now().nanoseconds() / 1000; }

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    // State
    geometry_msgs::msg::Twist velocity_;
    rclcpp::Time last_cmd_time_;
    std::optional<rclcpp::Time> hover_start_time_;
    bool has_cmd_ = false;
    bool has_pos_ = false;
    bool offboard_mode_ = false;

    // PX4 state
    uint8_t nav_state_ = 0;
    uint8_t arm_state_ = 0;
    bool flight_check_ = false;
    bool failsafe_ = false;
    bool last_failsafe_ = false;
    float pos_x_ = 0, pos_y_ = 0, pos_z_ = 0;
    float hover_x_ = 0, hover_y_ = 0, hover_z_ = 0;
    float yaw_ = 0;

    // Parameters
    double takeoff_altitude_ = 1.5;
    double cmd_timeout_ = 0.5;
    double hover_timeout_ = 30.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}