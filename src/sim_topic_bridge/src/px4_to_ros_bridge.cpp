#include "include/px4_to_ros_bridge.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

namespace px4_to_ros_bridge
{

PX4ToROS::PX4ToROS() : Node("px4_to_ros_bridge")
{
    rclcpp::QoS px4_qos(1);
    px4_qos.best_effort();
    
    // Subscribers - std::bind ile normal fonksiyona bağla
    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", px4_qos,
        std::bind(&PX4ToROS::positionCallback, this, _1));
    
    att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", px4_qos,
        std::bind(&PX4ToROS::attitudeCallback, this, _1));
    
    // Publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    
    // Static TF: map -> odom
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.rotation.w = 1.0;
    static_tf_pub_->sendTransform(t);
    
    // Timer - 50Hz
    timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&PX4ToROS::timerCallback, this));
    
    RCLCPP_INFO(get_logger(), "PX4 to ROS started");
}

void PX4ToROS::positionCallback(px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (msg->xy_valid && msg->z_valid) {
        pos_ = msg;
    }
}

void PX4ToROS::attitudeCallback(px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    att_ = msg;
}

void PX4ToROS::timerCallback()
{
    if (!pos_ || !att_) return;
    
    rclcpp::Time stamp = now();
    
    // NED -> ENU pozisyon dönüşümü
    // NED: x=North, y=East, z=Down
    // ENU: x=East, y=North, z=Up
    double x = pos_->x;   // North -> North (sonra rotate edilecek)
    double y = pos_->y;   // East -> East
    double z = -pos_->z;  // Down -> Up
    
    // NED -> ENU quaternion dönüşümü
    // 90 derece Z ekseni etrafında rotasyon + X ekseni etrafında 180 derece
    double qw_ned = att_->q[0];
    double qx_ned = att_->q[1];
    double qy_ned = att_->q[2];
    double qz_ned = att_->q[3];
    
    // Basit dönüşüm: NED->ENU için (x,y,z,w) -> (y,x,-z,w)
    double qw = qw_ned;
    double qx = qy_ned;
    double qy = qx_ned;
    double qz = -qz_ned;
    
    // Pozisyonu da ENU'ya çevir
    double enu_x = x;   // East
    double enu_y = -y;   // North
    double enu_z = z;   // Up
    
    // Odometry mesajı
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = enu_x;
    odom.pose.pose.position.y = enu_y;
    odom.pose.pose.position.z = enu_z;
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy;
    odom.pose.pose.orientation.z = qz;
    odom.twist.twist.linear.x = pos_->vx;   // East velocity
    odom.twist.twist.linear.y = pos_->vy;   // North velocity
    odom.twist.twist.linear.z = -pos_->vz;
    odom_pub_->publish(odom);
    
    // TF: odom -> base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = enu_x;
    tf.transform.translation.y = enu_y;
    tf.transform.translation.z = enu_z;
    tf.transform.rotation.w = qw;
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    tf_pub_->sendTransform(tf);
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_to_ros_bridge::PX4ToROS>());
    rclcpp::shutdown();
    return 0;
}