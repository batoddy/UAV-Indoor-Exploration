#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>
#include <mutex>
#include <atomic>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/camera_info.pb.h>

class DepthBridge : public rclcpp::Node
{
public:
    DepthBridge() : Node("depth_bridge")
    {
        // use_sim_time desteği
        this->declare_parameter("use_sim_time", true);

        // Sensor data için uygun QoS (best effort, volatile, küçük queue)
        auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

        // Publishers
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth", sensor_qos);
        points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points", sensor_qos);
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", sensor_qos);

        // Gazebo subscribers
        gz_node_.Subscribe("/depth_camera", &DepthBridge::onDepth, this);
        gz_node_.Subscribe("/depth_camera/points", &DepthBridge::onPoints, this);
        gz_node_.Subscribe("/camera_info", &DepthBridge::onCameraInfo, this);

        RCLCPP_INFO(get_logger(), "Depth bridge started (optimized)");
    }

private:
    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

    // Thread safety
    std::mutex depth_mutex_;
    std::mutex points_mutex_;

    // Duplicate frame filtering
    std::atomic<int64_t> last_depth_stamp_{0};
    std::atomic<int64_t> last_points_stamp_{0};

    // Pre-cached PointCloud fields (sadece bir kez oluştur)
    std::vector<sensor_msgs::msg::PointField> cached_fields_;
    bool fields_cached_ = false;
    std::mutex fields_mutex_;

    // Pre-allocated depth buffer
    std::vector<uint8_t> depth_buffer_;

    // Gazebo timestamp'ini ROS timestamp'ine çevir
    inline rclcpp::Time gzToRosTime(const gz::msgs::Header &header)
    {
        if (header.has_stamp()) {
            return rclcpp::Time(
                header.stamp().sec(),
                static_cast<uint32_t>(header.stamp().nsec()),
                RCL_ROS_TIME
            );
        }
        return now();
    }

    // Timestamp'i int64 olarak al (karşılaştırma için)
    inline int64_t getStampNs(const gz::msgs::Header &header)
    {
        if (header.has_stamp()) {
            return header.stamp().sec() * 1000000000LL + header.stamp().nsec();
        }
        return 0;
    }

    void onDepth(const gz::msgs::Image &msg)
    {
        // Subscriber yoksa işlem yapma
        if (depth_pub_->get_subscription_count() == 0) return;

        // Duplicate frame kontrolü
        int64_t stamp_ns = getStampNs(msg.header());
        int64_t last = last_depth_stamp_.load(std::memory_order_relaxed);
        if (stamp_ns <= last) return;
        last_depth_stamp_.store(stamp_ns, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(depth_mutex_);

        sensor_msgs::msg::Image ros_msg;
        ros_msg.header.stamp = gzToRosTime(msg.header());
        ros_msg.header.frame_id = "camera_optical_frame";
        ros_msg.width = msg.width();
        ros_msg.height = msg.height();
        ros_msg.encoding = "16UC1";
        ros_msg.step = msg.width() * 2;
        ros_msg.is_bigendian = false;

        const float* in = reinterpret_cast<const float*>(msg.data().data());
        size_t count = msg.width() * msg.height();

        // Pre-allocated buffer kullan
        size_t required_size = count * 2;
        if (depth_buffer_.size() < required_size) {
            depth_buffer_.resize(required_size);
        }

        uint16_t* out = reinterpret_cast<uint16_t*>(depth_buffer_.data());

        // Vectorized-friendly loop
        for (size_t i = 0; i < count; ++i) {
            float d = in[i];
            out[i] = (d > 0.0f && d < 65.535f) ?
                     static_cast<uint16_t>(d * 1000.0f) : 0;
        }

        ros_msg.data.assign(depth_buffer_.begin(), depth_buffer_.begin() + required_size);
        depth_pub_->publish(std::move(ros_msg));
    }

    void onPoints(const gz::msgs::PointCloudPacked &msg)
    {
        // Subscriber yoksa işlem yapma
        if (points_pub_->get_subscription_count() == 0) return;

        // Duplicate frame kontrolü
        int64_t stamp_ns = getStampNs(msg.header());
        int64_t last = last_points_stamp_.load(std::memory_order_relaxed);
        if (stamp_ns <= last) return;
        last_points_stamp_.store(stamp_ns, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(points_mutex_);

        sensor_msgs::msg::PointCloud2 ros_msg;
        ros_msg.header.stamp = gzToRosTime(msg.header());
        ros_msg.header.frame_id = "camera_optical_frame";
        ros_msg.width = msg.width();
        ros_msg.height = msg.height();
        ros_msg.point_step = msg.point_step();
        ros_msg.row_step = msg.row_step();
        ros_msg.is_dense = true;

        // Field'ları cache'le (sadece bir kez oluştur)
        {
            std::lock_guard<std::mutex> flock(fields_mutex_);
            if (!fields_cached_ && msg.field_size() > 0) {
                cached_fields_.reserve(msg.field_size());
                for (const auto &f : msg.field()) {
                    sensor_msgs::msg::PointField pf;
                    pf.name = f.name();
                    pf.offset = f.offset();
                    pf.count = f.count();
                    switch (f.datatype()) {
                        case gz::msgs::PointCloudPacked::Field::FLOAT32:
                            pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            break;
                        case gz::msgs::PointCloudPacked::Field::FLOAT64:
                            pf.datatype = sensor_msgs::msg::PointField::FLOAT64;
                            break;
                        case gz::msgs::PointCloudPacked::Field::INT32:
                            pf.datatype = sensor_msgs::msg::PointField::INT32;
                            break;
                        case gz::msgs::PointCloudPacked::Field::UINT32:
                            pf.datatype = sensor_msgs::msg::PointField::UINT32;
                            break;
                        default:
                            pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
                    }
                    cached_fields_.push_back(pf);
                }
                fields_cached_ = true;
            }
            ros_msg.fields = cached_fields_;
        }

        // Zero-copy olmasa da, move semantics kullan
        const auto& data = msg.data();
        ros_msg.data.assign(data.begin(), data.end());

        points_pub_->publish(std::move(ros_msg));
    }

    void onCameraInfo(const gz::msgs::CameraInfo &msg)
    {
        // Subscriber yoksa işlem yapma
        if (info_pub_->get_subscription_count() == 0) return;

        sensor_msgs::msg::CameraInfo ros_msg;
        ros_msg.header.stamp = gzToRosTime(msg.header());
        ros_msg.header.frame_id = "camera_optical_frame";
        ros_msg.width = msg.width();
        ros_msg.height = msg.height();
        ros_msg.distortion_model = "plumb_bob";

        if (msg.has_intrinsics()) {
            const auto &k = msg.intrinsics().k();
            for (int i = 0; i < 9 && i < k.size(); ++i)
                ros_msg.k[i] = k.Get(i);
        }

        if (msg.has_projection()) {
            const auto &p = msg.projection().p();
            for (int i = 0; i < 12 && i < p.size(); ++i)
                ros_msg.p[i] = p.Get(i);
        }

        ros_msg.r[0] = ros_msg.r[4] = ros_msg.r[8] = 1.0;

        info_pub_->publish(std::move(ros_msg));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthBridge>());
    rclcpp::shutdown();
    return 0;
}
