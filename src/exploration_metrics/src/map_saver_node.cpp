/**
 * @file map_saver_node.cpp
 * @brief Node for saving current OctoMap and OccupancyGrid as ground truth files
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <std_srvs/srv/trigger.hpp>

#include <fstream>
#include <filesystem>
#include <iomanip>
#include <ctime>

using namespace std::chrono_literals;

class MapSaverNode : public rclcpp::Node
{
public:
    MapSaverNode() : Node("map_saver_node")
    {
        // Declare parameters
        declare_parameter("save_directory", "/home/batoddy/uav_ws/src/exploration_metrics/ground_truth");
        declare_parameter("map_topic", "/projected_map");
        declare_parameter("octomap_topic", "/octomap_binary");
        declare_parameter("auto_timestamp", true);

        // Get parameters
        save_directory_ = get_parameter("save_directory").as_string();
        map_topic_ = get_parameter("map_topic").as_string();
        octomap_topic_ = get_parameter("octomap_topic").as_string();
        auto_timestamp_ = get_parameter("auto_timestamp").as_bool();

        // Create save directory if it doesn't exist
        if (!std::filesystem::exists(save_directory_)) {
            std::filesystem::create_directories(save_directory_);
            RCLCPP_INFO(get_logger(), "Created save directory: %s", save_directory_.c_str());
        }

        // Create subscribers
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, 10,
            std::bind(&MapSaverNode::mapCallback, this, std::placeholders::_1));

        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, 10,
            std::bind(&MapSaverNode::octomapCallback, this, std::placeholders::_1));

        // Create services
        save_gt_srv_ = create_service<std_srvs::srv::Trigger>(
            "/exploration_metrics/save_ground_truth",
            std::bind(&MapSaverNode::saveGroundTruthCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        save_octomap_srv_ = create_service<std_srvs::srv::Trigger>(
            "/exploration_metrics/save_octomap",
            std::bind(&MapSaverNode::saveOctomapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        save_occupancy_grid_srv_ = create_service<std_srvs::srv::Trigger>(
            "/exploration_metrics/save_occupancy_grid",
            std::bind(&MapSaverNode::saveOccupancyGridCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Map Saver Node initialized");
        RCLCPP_INFO(get_logger(), "  Save directory: %s", save_directory_.c_str());
        RCLCPP_INFO(get_logger(), "  Map topic: %s", map_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  OctoMap topic: %s", octomap_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Services:");
        RCLCPP_INFO(get_logger(), "  /exploration_metrics/save_ground_truth - Save both maps");
        RCLCPP_INFO(get_logger(), "  /exploration_metrics/save_octomap - Save OctoMap only");
        RCLCPP_INFO(get_logger(), "  /exploration_metrics/save_occupancy_grid - Save OccupancyGrid only");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = msg;
        map_received_ = true;
    }

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        current_octomap_msg_ = msg;
        octomap_received_ = true;
    }

    std::string getTimestampString()
    {
        if (!auto_timestamp_) {
            return "";
        }
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "_%Y%m%d_%H%M%S");
        return ss.str();
    }

    bool saveOctomap(const std::string& filename)
    {
        if (!octomap_received_ || !current_octomap_msg_) {
            RCLCPP_ERROR(get_logger(), "No OctoMap data received yet");
            return false;
        }

        // Convert message to OcTree
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*current_octomap_msg_);
        if (!tree) {
            RCLCPP_ERROR(get_logger(), "Failed to convert OctoMap message");
            return false;
        }

        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        if (!octree) {
            RCLCPP_ERROR(get_logger(), "Failed to cast to OcTree");
            delete tree;
            return false;
        }

        // Save as binary file
        std::string full_path = save_directory_ + "/" + filename;
        bool success = octree->writeBinary(full_path);
        delete tree;

        if (success) {
            RCLCPP_INFO(get_logger(), "Saved OctoMap to: %s", full_path.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to save OctoMap to: %s", full_path.c_str());
        }

        return success;
    }

    bool saveOccupancyGrid(const std::string& base_filename)
    {
        if (!map_received_ || !current_map_) {
            RCLCPP_ERROR(get_logger(), "No OccupancyGrid data received yet");
            return false;
        }

        std::string pgm_path = save_directory_ + "/" + base_filename + ".pgm";
        std::string yaml_path = save_directory_ + "/" + base_filename + ".yaml";

        // Save PGM file
        if (!savePgm(pgm_path)) {
            return false;
        }

        // Save YAML file
        if (!saveYaml(yaml_path, base_filename + ".pgm")) {
            return false;
        }

        RCLCPP_INFO(get_logger(), "Saved OccupancyGrid to: %s", yaml_path.c_str());
        return true;
    }

    bool savePgm(const std::string& filename)
    {
        int width = current_map_->info.width;
        int height = current_map_->info.height;

        std::ofstream pgm_file(filename, std::ios::binary);
        if (!pgm_file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open PGM file for writing: %s", filename.c_str());
            return false;
        }

        // Write PGM header
        pgm_file << "P5\n";
        pgm_file << "# Ground truth map saved by exploration_metrics\n";
        pgm_file << width << " " << height << "\n";
        pgm_file << "255\n";

        // Write pixel data (flip Y axis for image format)
        for (int y = height - 1; y >= 0; --y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                int8_t val = current_map_->data[idx];

                unsigned char pixel;
                if (val == -1) {
                    // Unknown -> gray (205)
                    pixel = 205;
                } else if (val == 0) {
                    // Free -> white (254)
                    pixel = 254;
                } else if (val == 100) {
                    // Occupied -> black (0)
                    pixel = 0;
                } else {
                    // Intermediate values
                    pixel = static_cast<unsigned char>((100 - val) * 255 / 100);
                }

                pgm_file.write(reinterpret_cast<char*>(&pixel), 1);
            }
        }

        pgm_file.close();
        return true;
    }

    bool saveYaml(const std::string& yaml_filename, const std::string& pgm_filename)
    {
        std::ofstream yaml_file(yaml_filename);
        if (!yaml_file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open YAML file for writing: %s", yaml_filename.c_str());
            return false;
        }

        yaml_file << std::fixed << std::setprecision(6);
        yaml_file << "image: " << pgm_filename << "\n";
        yaml_file << "resolution: " << current_map_->info.resolution << "\n";
        yaml_file << "origin: [" << current_map_->info.origin.position.x << ", "
                  << current_map_->info.origin.position.y << ", "
                  << current_map_->info.origin.position.z << "]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: 0.65\n";
        yaml_file << "free_thresh: 0.196\n";

        yaml_file.close();
        return true;
    }

    void saveGroundTruthCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string timestamp = getTimestampString();
        bool octomap_ok = true;
        bool occupancy_grid_ok = true;

        // Save OctoMap
        if (octomap_received_) {
            std::string octomap_filename = "ground_truth" + timestamp + ".bt";
            octomap_ok = saveOctomap(octomap_filename);
        } else {
            RCLCPP_WARN(get_logger(), "OctoMap not received, skipping");
        }

        // Save OccupancyGrid
        if (map_received_) {
            std::string occupancy_grid_filename = "ground_truth" + timestamp;
            occupancy_grid_ok = saveOccupancyGrid(occupancy_grid_filename);
        } else {
            RCLCPP_WARN(get_logger(), "OccupancyGrid not received, skipping");
        }

        if (octomap_ok && occupancy_grid_ok) {
            response->success = true;
            response->message = "Ground truth maps saved successfully";
        } else {
            response->success = false;
            response->message = "Some maps failed to save";
        }
    }

    void saveOctomapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string timestamp = getTimestampString();
        std::string filename = "ground_truth" + timestamp + ".bt";

        if (saveOctomap(filename)) {
            response->success = true;
            response->message = "OctoMap saved: " + filename;
        } else {
            response->success = false;
            response->message = "Failed to save OctoMap";
        }
    }

    void saveOccupancyGridCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string timestamp = getTimestampString();
        std::string base_filename = "ground_truth" + timestamp;

        if (saveOccupancyGrid(base_filename)) {
            response->success = true;
            response->message = "OccupancyGrid saved: " + base_filename + ".yaml";
        } else {
            response->success = false;
            response->message = "Failed to save OccupancyGrid";
        }
    }

    // Parameters
    std::string save_directory_;
    std::string map_topic_;
    std::string octomap_topic_;
    bool auto_timestamp_;

    // Current data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    octomap_msgs::msg::Octomap::SharedPtr current_octomap_msg_;
    bool map_received_ = false;
    bool octomap_received_ = false;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_gt_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_octomap_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_occupancy_grid_srv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaverNode>());
    rclcpp::shutdown();
    return 0;
}
