#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/camera_info.pb.h>

class DepthBridge : public rclcpp::Node
{
public:
    DepthBridge() : Node("depth_bridge")
    {
        // Publishers
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth", 10);
        points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points", 10);
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

        // Gazebo subscribers
        gz_node_.Subscribe("/depth_camera", &DepthBridge::onDepth, this);
        gz_node_.Subscribe("/depth_camera/points", &DepthBridge::onPoints, this);
        gz_node_.Subscribe("/camera_info", &DepthBridge::onCameraInfo, this);

        RCLCPP_INFO(get_logger(), "Depth bridge started");
    }

private:
    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

    void onDepth(const gz::msgs::Image &msg)
    {
        sensor_msgs::msg::Image ros_msg;
        ros_msg.header.stamp = now();
        ros_msg.header.frame_id = "camera_optical_frame";
        ros_msg.width = msg.width();
        ros_msg.height = msg.height();
        
        // Float32 -> 16UC1 (mm) RViz uyumlu
        ros_msg.encoding = "16UC1";
        ros_msg.step = msg.width() * 2;
        ros_msg.is_bigendian = false;
        
        const float* in = reinterpret_cast<const float*>(msg.data().data());
        size_t count = msg.width() * msg.height();
        ros_msg.data.resize(count * 2);
        uint16_t* out = reinterpret_cast<uint16_t*>(ros_msg.data.data());
        
        for (size_t i = 0; i < count; i++) {
            float d = in[i];
            out[i] = (std::isfinite(d) && d > 0) ? 
                     static_cast<uint16_t>(std::min(d * 1000.0f, 65535.0f)) : 0;
        }
        
        depth_pub_->publish(ros_msg);
    }

    void onPoints(const gz::msgs::PointCloudPacked &msg)
    {
        sensor_msgs::msg::PointCloud2 ros_msg;
        ros_msg.header.stamp = now();
        ros_msg.header.frame_id = "camera_optical_frame";
        ros_msg.width = msg.width();
        ros_msg.height = msg.height();
        ros_msg.point_step = msg.point_step();
        ros_msg.row_step = msg.row_step();
        ros_msg.is_dense = true;

        for (const auto &f : msg.field()) {
            sensor_msgs::msg::PointField pf;
            pf.name = f.name();
            pf.offset = f.offset();
            switch (f.datatype()) {
    case gz::msgs::PointCloudPacked::Field::FLOAT32:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT32;  // 7
        break;
    case gz::msgs::PointCloudPacked::Field::FLOAT64:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT64;  // 8
        break;
    case gz::msgs::PointCloudPacked::Field::INT32:
        pf.datatype = sensor_msgs::msg::PointField::INT32;    // 5
        break;
    case gz::msgs::PointCloudPacked::Field::UINT32:
        pf.datatype = sensor_msgs::msg::PointField::UINT32;   // 6
        break;
    default:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
}

            pf.count = f.count();
            ros_msg.fields.push_back(pf);
        }

        ros_msg.data.assign(msg.data().begin(), msg.data().end());
        points_pub_->publish(ros_msg);
    }

    void onCameraInfo(const gz::msgs::CameraInfo &msg)
    {
        sensor_msgs::msg::CameraInfo ros_msg;
        ros_msg.header.stamp = now();
        ros_msg.header.frame_id = "camera_optical_frame";
        ros_msg.width = msg.width();
        ros_msg.height = msg.height();
        ros_msg.distortion_model = "plumb_bob";

        // Intrinsic matrix K
        if (msg.has_intrinsics()) {
            const auto &k = msg.intrinsics().k();
            for (int i = 0; i < 9 && i < k.size(); i++)
                ros_msg.k[i] = k.Get(i);
        }

        // Projection matrix P
        if (msg.has_projection()) {
            const auto &p = msg.projection().p();
            for (int i = 0; i < 12 && i < p.size(); i++)
                ros_msg.p[i] = p.Get(i);
        }

        // Rectification matrix R (identity)
        ros_msg.r[0] = ros_msg.r[4] = ros_msg.r[8] = 1.0;

        info_pub_->publish(ros_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthBridge>());
    rclcpp::shutdown();
    return 0;
}