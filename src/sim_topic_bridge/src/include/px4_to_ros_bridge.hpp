#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace px4_to_ros_bridge
{

class PX4ToROS : public rclcpp::Node
{
public:
    PX4ToROS();

private:
    // Callback fonksiyonları
    void positionCallback(px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void attitudeCallback(px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void timerCallback();
    
    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr att_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Veriler
    px4_msgs::msg::VehicleLocalPosition::SharedPtr pos_;
    px4_msgs::msg::VehicleAttitude::SharedPtr att_;
};

}