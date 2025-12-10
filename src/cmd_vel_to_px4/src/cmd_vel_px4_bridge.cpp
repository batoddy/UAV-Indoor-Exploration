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
        declare_parameter<double>("takeoff_altitude");
        takeoff_altitude_ = get_parameter("takeoff_altitude").as_double();

        rclcpp::QoS px4_qos(10);
        px4_qos.best_effort();

        // Subscribers
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                velocity_ = *msg;
                last_cmd_time_ = now();
                has_cmd_ = true;
            });

        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4_qos,
            [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                nav_state_ = msg->nav_state;
                arm_state_ = msg->arming_state;
                flight_check_ = msg->pre_flight_checks_pass;
                failsafe_ = msg->failsafe;
            });

        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", px4_qos,
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
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
        cmd_timer_ = create_wall_timer(20ms, [this]() { cmdLoop(); });

        RCLCPP_INFO(get_logger(), "Offboard Control started");
        RCLCPP_INFO(get_logger(), "  Takeoff altitude: %.1f m", takeoff_altitude_);
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
                // Offboard mode'a geçmeden önce setpoint gönder
                publishOffboardMode(true);
                publishPositionSetpoint(-takeoff_altitude_);
                
                if (++counter_ >= 20) {  // 2 saniye
                    setOffboardMode();
                    state_ = TAKEOFF;
                    counter_ = 0;
                    RCLCPP_INFO(get_logger(), "State: TAKEOFF to %.1fm", takeoff_altitude_);
                }
                break;

            case TAKEOFF:
                publishOffboardMode(true);
                publishPositionSetpoint(-takeoff_altitude_);
                
                // Yüksekliğe ulaştı mı? (NED: z negatif = yukarı)
                if (-pos_z_ >= takeoff_altitude_ * 0.9) {
                    state_ = OFFBOARD;
                    offboard_mode_ = true;
                    RCLCPP_INFO(get_logger(), "State: OFFBOARD - Ready for cmd_vel!");
                }
                break;

            case OFFBOARD:
                if (!flight_check_ || arm_state_ != 2 || failsafe_) {
                    state_ = IDLE;
                    offboard_mode_ = false;
                    RCLCPP_WARN(get_logger(), "Back to IDLE");
                }
                break;
        }
    }

    void cmdLoop()
    {
        if (!offboard_mode_ || !has_pos_) return;

        // Velocity control mode
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.timestamp = timestamp();
        mode_msg.position = false;
        mode_msg.velocity = true;
        offboard_pub_->publish(mode_msg);

        bool cmd_active = has_cmd_ && (now() - last_cmd_time_).seconds() < 0.5;

        px4_msgs::msg::TrajectorySetpoint traj_msg;
        traj_msg.timestamp = timestamp();

        if (cmd_active) {
            // Body -> World frame dönüşümü
            float vx = velocity_.linear.x;
            float vy = -velocity_.linear.y;
            float cos_yaw = std::cos(yaw_);
            float sin_yaw = std::sin(yaw_);

            traj_msg.velocity[0] = vx * cos_yaw - vy * sin_yaw;
            traj_msg.velocity[1] = vx * sin_yaw + vy * cos_yaw;
            traj_msg.velocity[2] = -velocity_.linear.z;
            traj_msg.yawspeed = -velocity_.angular.z;
        } else {
            // Hover
            traj_msg.velocity[0] = 0.0;
            traj_msg.velocity[1] = 0.0;
            traj_msg.velocity[2] = 0.0;
            traj_msg.yawspeed = 0.0;
        }

        traj_msg.position = {NAN, NAN, NAN};
        traj_msg.yaw = NAN;
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

    // Subs
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

    // Pubs
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    // State
    geometry_msgs::msg::Twist velocity_;
    rclcpp::Time last_cmd_time_;
    bool has_cmd_ = false;
    bool has_pos_ = false;
    bool offboard_mode_ = false;

    // PX4
    uint8_t nav_state_ = 0;
    uint8_t arm_state_ = 0;
    bool flight_check_ = false;
    bool failsafe_ = false;
    float pos_z_ = 0;
    float yaw_ = 0;

    double takeoff_altitude_ = 5.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}

// ```## Akış```IDLE → ARMING → TAKEOFF_PREP (2sn setpoint) → TAKEOFF (yükseklik kontrol) → OFFBOARD (cmd_vel)