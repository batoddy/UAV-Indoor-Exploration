/**
 * @file exploration_metrics_node.cpp
 * @brief Node for comparing current exploration map with ground truth and logging motion metrics
 *
 * Features:
 *   - Ground truth comparison (OccupancyGrid and/or OctoMap)
 *   - Motion telemetry logging (velocity, acceleration, yaw, path length)
 *   - Automatic start/stop with /exploration/start and /exploration/stop
 *   - Map-based directory organization with timestamped log files
 *
 * Usage:
 *   ros2 launch exploration_metrics exploration_metrics.launch.py map:=octomaze
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "exploration_metrics/msg/exploration_metrics.hpp"

// Include TelemetryStatus from exploration_planner
#include "exploration_planner/msg/telemetry_status.hpp"
#include "exploration_planner/msg/exploration_status.hpp"

#include <fstream>
#include <chrono>
#include <cmath>
#include <unordered_set>
#include <filesystem>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class ExplorationMetricsNode : public rclcpp::Node
{
public:
    ExplorationMetricsNode() : Node("exploration_metrics_node")
    {
        // Declare parameters
        declare_parameter("comparison_mode", "both");
        declare_parameter("map_name", "default");
        declare_parameter("base_path", "");
        declare_parameter("publish_rate", 1.0);
        declare_parameter("logging_enabled", true);
        declare_parameter("occupied_threshold", 65);
        declare_parameter("free_threshold", 25);
        declare_parameter("position_tolerance", 0.01);
        declare_parameter("map_topic", "/projected_map");
        declare_parameter("octomap_topic", "/octomap_binary");
        declare_parameter("ema_alpha", 0.1);
        declare_parameter("telemetry_log_rate", 10.0);

        // Get parameters
        comparison_mode_ = get_parameter("comparison_mode").as_string();
        map_name_ = get_parameter("map_name").as_string();
        base_path_ = get_parameter("base_path").as_string();
        publish_rate_ = get_parameter("publish_rate").as_double();
        logging_enabled_ = get_parameter("logging_enabled").as_bool();
        occupied_threshold_ = get_parameter("occupied_threshold").as_int();
        free_threshold_ = get_parameter("free_threshold").as_int();
        position_tolerance_ = get_parameter("position_tolerance").as_double();
        map_topic_ = get_parameter("map_topic").as_string();
        octomap_topic_ = get_parameter("octomap_topic").as_string();
        ema_alpha_ = get_parameter("ema_alpha").as_double();
        telemetry_log_rate_ = get_parameter("telemetry_log_rate").as_double();

        // Setup paths based on map_name
        setupPaths();

        // Load ground truth maps
        loadGroundTruthMaps();

        // Create publishers
        metrics_pub_ = create_publisher<exploration_metrics::msg::ExplorationMetrics>(
            "/exploration/metrics", 10);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/exploration/metrics_markers", 10);

        // Create subscribers for maps
        if (comparison_mode_ == "occupancy_grid" || comparison_mode_ == "both") {
            map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                map_topic_, 10,
                std::bind(&ExplorationMetricsNode::mapCallback, this, std::placeholders::_1));
        }

        if (comparison_mode_ == "octomap" || comparison_mode_ == "both") {
            octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
                octomap_topic_, 10,
                std::bind(&ExplorationMetricsNode::octomapCallback, this, std::placeholders::_1));
        }

        // Subscribe to telemetry from exploration_planner
        telemetry_sub_ = create_subscription<exploration_planner::msg::TelemetryStatus>(
            "/exploration/telemetry", 10,
            std::bind(&ExplorationMetricsNode::telemetryCallback, this, std::placeholders::_1));

        // Subscribe to exploration status (for auto start/stop)
        exploration_status_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
            "/exploration/global_tour", 10,
            std::bind(&ExplorationMetricsNode::explorationStatusCallback, this, std::placeholders::_1));

        // Create services
        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "/exploration_metrics/reset",
            std::bind(&ExplorationMetricsNode::resetCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        new_log_srv_ = create_service<std_srvs::srv::Trigger>(
            "/exploration_metrics/new_log",
            std::bind(&ExplorationMetricsNode::newLogCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Create timer for publishing metrics
        double period_ms = 1000.0 / publish_rate_;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&ExplorationMetricsNode::publishMetrics, this));

        // Record start time (wall clock)
        start_time_wall_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(), "Exploration Metrics Node initialized");
        RCLCPP_INFO(get_logger(), "  Map name: %s", map_name_.c_str());
        RCLCPP_INFO(get_logger(), "  Comparison mode: %s", comparison_mode_.c_str());
        RCLCPP_INFO(get_logger(), "  Ground truth path: %s", gt_occupancy_grid_path_.c_str());
        RCLCPP_INFO(get_logger(), "  Log directory: %s", log_dir_.c_str());
        RCLCPP_INFO(get_logger(), "  Logging: %s (auto-triggered by /exploration/start)",
                    logging_enabled_ ? "enabled" : "disabled");
        RCLCPP_INFO(get_logger(), "  Waiting for telemetry on: /exploration/telemetry");
        RCLCPP_INFO(get_logger(), "  Waiting for exploration status on: /exploration/global_tour");
    }

    ~ExplorationMetricsNode()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void setupPaths()
    {
        // Default base path if not provided
        if (base_path_.empty()) {
            base_path_ = "/home/batoddy/uav_ws";
        }

        // Ground truth path: base_path/src/exploration_metrics/ground_truth/map_name/map_name.yaml
        gt_occupancy_grid_path_ = base_path_ + "/src/exploration_metrics/ground_truth/" +
                                   map_name_ + "/" + map_name_ + ".yaml";
        gt_octomap_path_ = base_path_ + "/src/exploration_metrics/ground_truth/" +
                           map_name_ + "/" + map_name_ + ".bt";

        // Log directory: base_path/metric_logs/map_name/
        log_dir_ = base_path_ + "/metric_logs/" + map_name_;

        // Create log directory if it doesn't exist
        std::filesystem::create_directories(log_dir_);

        RCLCPP_INFO(get_logger(), "Paths configured for map: %s", map_name_.c_str());
    }

    std::string generateTimestampedFilename()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time_t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        return oss.str();
    }

    void loadGroundTruthMaps()
    {
        // Load ground truth OctoMap
        if (comparison_mode_ == "octomap" || comparison_mode_ == "both") {
            if (std::filesystem::exists(gt_octomap_path_)) {
                gt_octomap_ = std::make_shared<octomap::OcTree>(gt_octomap_path_);
                if (gt_octomap_) {
                    gt_known_voxels_ = 0;
                    for (auto it = gt_octomap_->begin_leafs(); it != gt_octomap_->end_leafs(); ++it) {
                        gt_known_voxels_++;
                    }
                    RCLCPP_INFO(get_logger(), "Loaded ground truth OctoMap: %s (%d voxels)",
                                gt_octomap_path_.c_str(), gt_known_voxels_);
                    gt_octomap_loaded_ = true;
                }
            } else {
                RCLCPP_WARN(get_logger(), "Ground truth OctoMap not found: %s", gt_octomap_path_.c_str());
            }
        }

        // Load ground truth OccupancyGrid
        if (comparison_mode_ == "occupancy_grid" || comparison_mode_ == "both") {
            if (std::filesystem::exists(gt_occupancy_grid_path_)) {
                loadOccupancyGridFromYaml(gt_occupancy_grid_path_);
            } else {
                RCLCPP_WARN(get_logger(), "Ground truth OccupancyGrid not found: %s",
                            gt_occupancy_grid_path_.c_str());
            }
        }
    }

    void loadOccupancyGridFromYaml(const std::string& yaml_path)
    {
        std::ifstream yaml_file(yaml_path);
        if (!yaml_file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open YAML file: %s", yaml_path.c_str());
            return;
        }

        std::string line;
        std::string pgm_path;
        double resolution = 0.05;
        double origin_x = 0.0, origin_y = 0.0;
        int negate = 0;
        double occupied_thresh = 0.65;
        double free_thresh = 0.196;

        while (std::getline(yaml_file, line)) {
            if (line.find("image:") != std::string::npos) {
                size_t pos = line.find(":");
                pgm_path = line.substr(pos + 1);
                pgm_path.erase(0, pgm_path.find_first_not_of(" \t"));
                pgm_path.erase(pgm_path.find_last_not_of(" \t") + 1);
            } else if (line.find("resolution:") != std::string::npos) {
                sscanf(line.c_str(), "resolution: %lf", &resolution);
            } else if (line.find("origin:") != std::string::npos) {
                sscanf(line.c_str(), "origin: [%lf, %lf", &origin_x, &origin_y);
            } else if (line.find("negate:") != std::string::npos) {
                sscanf(line.c_str(), "negate: %d", &negate);
            } else if (line.find("occupied_thresh:") != std::string::npos) {
                sscanf(line.c_str(), "occupied_thresh: %lf", &occupied_thresh);
            } else if (line.find("free_thresh:") != std::string::npos) {
                sscanf(line.c_str(), "free_thresh: %lf", &free_thresh);
            }
        }
        yaml_file.close();

        // Handle relative path
        if (pgm_path[0] != '/') {
            std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();
            pgm_path = (yaml_dir / pgm_path).string();
        }

        if (!loadPgmFile(pgm_path, resolution, origin_x, origin_y, negate,
                         occupied_thresh, free_thresh)) {
            RCLCPP_ERROR(get_logger(), "Failed to load PGM file: %s", pgm_path.c_str());
            return;
        }

        gt_occupancy_grid_loaded_ = true;
        RCLCPP_INFO(get_logger(), "Loaded ground truth OccupancyGrid: %s (%d known cells)",
                    yaml_path.c_str(), gt_known_cells_);
    }

    bool loadPgmFile(const std::string& pgm_path, double resolution,
                     double origin_x, double origin_y, int negate,
                     double occupied_thresh, double free_thresh)
    {
        std::ifstream pgm_file(pgm_path, std::ios::binary);
        if (!pgm_file.is_open()) return false;

        std::string magic;
        pgm_file >> magic;
        if (magic != "P5") {
            RCLCPP_ERROR(get_logger(), "Invalid PGM format (expected P5): %s", magic.c_str());
            return false;
        }

        char c;
        pgm_file.get(c);
        while (pgm_file.peek() == '#') {
            std::string comment;
            std::getline(pgm_file, comment);
        }

        int width, height, max_val;
        pgm_file >> width >> height >> max_val;
        pgm_file.get(c);

        gt_occupancy_grid_.info.width = width;
        gt_occupancy_grid_.info.height = height;
        gt_occupancy_grid_.info.resolution = resolution;
        gt_occupancy_grid_.info.origin.position.x = origin_x;
        gt_occupancy_grid_.info.origin.position.y = origin_y;
        gt_occupancy_grid_.data.resize(width * height);

        std::vector<unsigned char> pixels(width * height);
        pgm_file.read(reinterpret_cast<char*>(pixels.data()), width * height);

        gt_known_cells_ = 0;
        for (int y = height - 1; y >= 0; --y) {
            for (int x = 0; x < width; ++x) {
                int src_idx = y * width + x;
                int dst_idx = (height - 1 - y) * width + x;

                unsigned char pixel = pixels[src_idx];
                if (negate) pixel = 255 - pixel;

                double p = static_cast<double>(pixel) / 255.0;

                if (p > occupied_thresh) {
                    gt_occupancy_grid_.data[dst_idx] = 0;
                    gt_known_cells_++;
                } else if (p < free_thresh) {
                    gt_occupancy_grid_.data[dst_idx] = 100;
                    gt_known_cells_++;
                } else {
                    gt_occupancy_grid_.data[dst_idx] = -1;
                }
            }
        }

        return true;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = msg;
        map_received_ = true;
    }

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            current_octomap_.reset(dynamic_cast<octomap::OcTree*>(tree));
            octomap_received_ = true;
        }
    }

    void telemetryCallback(const exploration_planner::msg::TelemetryStatus::SharedPtr msg)
    {
        if (!telemetry_received_) {
            RCLCPP_INFO(get_logger(), "Telemetry connected - receiving motion data");
        }

        latest_telemetry_ = *msg;
        telemetry_received_ = true;

        // Log telemetry if exploration is active
        if (exploration_active_ && logging_enabled_ && csv_file_.is_open()) {
            logTelemetryToCsv(*msg);
        }
    }

    void explorationStatusCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
    {
        uint8_t prev_state = current_exploration_state_;
        current_exploration_state_ = msg->state;

        // Detect state transitions
        bool was_active = (prev_state == exploration_planner::msg::ExplorationStatus::EXPLORING ||
                          prev_state == exploration_planner::msg::ExplorationStatus::MOVING_TO_VIEWPOINT ||
                          prev_state == exploration_planner::msg::ExplorationStatus::REFINING);

        bool is_active = (current_exploration_state_ == exploration_planner::msg::ExplorationStatus::EXPLORING ||
                         current_exploration_state_ == exploration_planner::msg::ExplorationStatus::MOVING_TO_VIEWPOINT ||
                         current_exploration_state_ == exploration_planner::msg::ExplorationStatus::REFINING);

        // Start logging when exploration becomes active
        if (!was_active && is_active && !exploration_active_) {
            startLogging();
        }

        // Stop logging when exploration becomes inactive
        if (was_active && !is_active && exploration_active_) {
            stopLogging();
        }
    }

    void startLogging()
    {
        exploration_active_ = true;
        exploration_start_time_ = std::chrono::steady_clock::now();

        // Reset counters
        last_exploration_percentage_ = -1.0;
        avg_exploration_rate_ = -1.0;
        telemetry_log_counter_ = 0;

        if (logging_enabled_) {
            initCsvFile();
        }

        RCLCPP_INFO(get_logger(), "=== Exploration STARTED - Logging to: %s ===",
                    current_log_file_.c_str());
    }

    void stopLogging()
    {
        if (exploration_active_) {
            auto end_time = std::chrono::steady_clock::now();
            double duration = std::chrono::duration<double>(end_time - exploration_start_time_).count();

            exploration_active_ = false;

            if (csv_file_.is_open()) {
                csv_file_.flush();
                csv_file_.close();
            }

            RCLCPP_INFO(get_logger(), "=== Exploration STOPPED - Duration: %.2f seconds ===", duration);
            RCLCPP_INFO(get_logger(), "Log saved to: %s", current_log_file_.c_str());
        }
    }

    void initCsvFile()
    {
        // Generate timestamped filename
        std::string timestamp = generateTimestampedFilename();
        current_log_file_ = log_dir_ + "/" + timestamp + ".csv";

        csv_file_.open(current_log_file_, std::ios::out | std::ios::trunc);
        if (csv_file_.is_open()) {
            // Header with all metrics
            csv_file_ << "timestamp,"
                      << "elapsed_time,"
                      // Motion metrics
                      << "velocity_instant,"
                      << "velocity_average,"
                      << "acceleration,"
                      << "yaw_rad,"
                      << "yaw_deg,"
                      << "path_traveled,"
                      // Exploration metrics
                      << "exploration_pct,"
                      << "2d_pct,"
                      << "3d_pct,"
                      << "exploration_rate,"
                      << "avg_exploration_rate"
                      << std::endl;

            RCLCPP_INFO(get_logger(), "CSV file created: %s", current_log_file_.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to create CSV file: %s", current_log_file_.c_str());
        }
    }

    void logTelemetryToCsv(const exploration_planner::msg::TelemetryStatus& telemetry)
    {
        // Rate limiting for telemetry logging
        telemetry_log_counter_++;
        int log_interval = static_cast<int>(30.0 / telemetry_log_rate_);  // Assuming 30Hz telemetry
        if (log_interval < 1) log_interval = 1;

        if (telemetry_log_counter_ % log_interval != 0) {
            return;
        }

        if (!csv_file_.is_open()) return;

        double elapsed = 0.0;
        if (exploration_active_) {
            auto now_wall = std::chrono::steady_clock::now();
            elapsed = std::chrono::duration<double>(now_wall - exploration_start_time_).count();
        }

        csv_file_ << std::fixed << std::setprecision(4)
                  << telemetry.header.stamp.sec << "."
                  << std::setfill('0') << std::setw(9) << telemetry.header.stamp.nanosec << ","
                  << std::setprecision(3)
                  << elapsed << ","
                  // Motion metrics
                  << telemetry.velocity_horizontal << ","
                  << telemetry.average_velocity << ","
                  << telemetry.acceleration_magnitude << ","
                  << telemetry.current_yaw << ","
                  << (telemetry.current_yaw * 180.0 / M_PI) << ","
                  << telemetry.total_path_traveled << ","
                  // Exploration metrics (from last published)
                  << last_exploration_pct_ << ","
                  << last_2d_pct_ << ","
                  << last_3d_pct_ << ","
                  << last_exploration_rate_ << ","
                  << avg_exploration_rate_
                  << std::endl;

        // Periodic flush
        if (telemetry_log_counter_ % 100 == 0) {
            csv_file_.flush();
        }
    }

    void publishMetrics()
    {
        exploration_metrics::msg::ExplorationMetrics metrics_msg;

        auto now_wall = std::chrono::steady_clock::now();
        auto now_system = std::chrono::system_clock::now();
        auto epoch_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now_system.time_since_epoch()).count();

        metrics_msg.header.stamp.sec = static_cast<int32_t>(epoch_time / 1000000000);
        metrics_msg.header.stamp.nanosec = static_cast<uint32_t>(epoch_time % 1000000000);
        metrics_msg.header.frame_id = "map";
        metrics_msg.comparison_mode = comparison_mode_;

        metrics_msg.tick = tick_counter_++;

        double elapsed = std::chrono::duration<double>(now_wall - start_time_wall_).count();
        metrics_msg.elapsed_time = elapsed;

        if (exploration_active_) {
            metrics_msg.exploration_duration = std::chrono::duration<double>(
                now_wall - exploration_start_time_).count();
        } else {
            metrics_msg.exploration_duration = -1.0;
        }

        // Motion metrics from telemetry
        if (telemetry_received_) {
            metrics_msg.velocity_instant = latest_telemetry_.velocity_horizontal;
            metrics_msg.velocity_average = latest_telemetry_.average_velocity;
            metrics_msg.acceleration = latest_telemetry_.acceleration_magnitude;
            metrics_msg.yaw_rad = latest_telemetry_.current_yaw;
            metrics_msg.yaw_deg = latest_telemetry_.current_yaw * 180.0 / M_PI;
            metrics_msg.path_traveled = latest_telemetry_.total_path_traveled;
        } else {
            metrics_msg.velocity_instant = 0.0;
            metrics_msg.velocity_average = 0.0;
            metrics_msg.acceleration = 0.0;
            metrics_msg.yaw_rad = 0.0;
            metrics_msg.yaw_deg = 0.0;
            metrics_msg.path_traveled = 0.0;
        }

        // Calculate OccupancyGrid metrics
        if ((comparison_mode_ == "occupancy_grid" || comparison_mode_ == "both") &&
            gt_occupancy_grid_loaded_ && map_received_) {
            calculateOccupancyGridMetrics(metrics_msg);
        }

        // Calculate OctoMap metrics
        if ((comparison_mode_ == "octomap" || comparison_mode_ == "both") &&
            gt_octomap_loaded_ && octomap_received_) {
            calculateOctomapMetrics(metrics_msg);
        }

        // Calculate overall exploration percentage
        if (comparison_mode_ == "both") {
            metrics_msg.exploration_percentage =
                (metrics_msg.exploration_2d_percentage + metrics_msg.exploration_3d_percentage) / 2.0;
        } else if (comparison_mode_ == "occupancy_grid") {
            metrics_msg.exploration_percentage = metrics_msg.exploration_2d_percentage;
        } else {
            metrics_msg.exploration_percentage = metrics_msg.exploration_3d_percentage;
        }

        // Calculate exploration rate
        double time_for_rate = exploration_active_ ? metrics_msg.exploration_duration : elapsed;

        // Initialize last values on first valid reading
        if (last_exploration_percentage_ < 0 && metrics_msg.exploration_percentage > 0) {
            last_exploration_percentage_ = metrics_msg.exploration_percentage;
            last_elapsed_time_ = time_for_rate;
        }

        if (last_exploration_percentage_ >= 0 && time_for_rate > last_elapsed_time_ + 0.01) {
            double dt = time_for_rate - last_elapsed_time_;
            double dp = metrics_msg.exploration_percentage - last_exploration_percentage_;
            double instant_rate = dp / dt;

            // Clamp to reasonable range
            instant_rate = std::max(-10.0, std::min(10.0, instant_rate));
            metrics_msg.exploration_rate = instant_rate;

            if (avg_exploration_rate_ < 0) {
                avg_exploration_rate_ = instant_rate;
            } else {
                avg_exploration_rate_ = ema_alpha_ * instant_rate +
                                        (1.0 - ema_alpha_) * avg_exploration_rate_;
            }
            metrics_msg.avg_exploration_rate = avg_exploration_rate_;
        } else {
            // Keep previous values instead of resetting to 0
            metrics_msg.exploration_rate = last_exploration_rate_;
            metrics_msg.avg_exploration_rate = avg_exploration_rate_ >= 0 ? avg_exploration_rate_ : 0.0;
        }

        // Store for CSV logging
        last_exploration_pct_ = metrics_msg.exploration_percentage;
        last_2d_pct_ = metrics_msg.exploration_2d_percentage;
        last_3d_pct_ = metrics_msg.exploration_3d_percentage;
        last_exploration_rate_ = metrics_msg.exploration_rate;

        last_exploration_percentage_ = metrics_msg.exploration_percentage;
        last_elapsed_time_ = time_for_rate;

        // Publish
        metrics_pub_->publish(metrics_msg);
        publishVisualizationMarkers(metrics_msg);

        // Log summary periodically
        if (exploration_active_ && static_cast<int>(metrics_msg.exploration_duration) % 10 == 0 &&
            metrics_msg.exploration_duration > 0.5) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                "[%s] 2D: %.1f%% | 3D: %.1f%% | Rate: %.2f%%/s | Path: %.1fm",
                map_name_.c_str(),
                metrics_msg.exploration_2d_percentage,
                metrics_msg.exploration_3d_percentage,
                metrics_msg.avg_exploration_rate,
                telemetry_received_ ? latest_telemetry_.total_path_traveled : 0.0);
        }
    }

    void calculateOccupancyGridMetrics(exploration_metrics::msg::ExplorationMetrics& msg)
    {
        if (!current_map_ || gt_known_cells_ == 0) return;

        int matched = 0;      // GT ile aynı konumda VE aynı değerde
        int mismatched = 0;   // GT ile aynı konumda AMA farklı değerde

        double gt_resolution = gt_occupancy_grid_.info.resolution;
        double gt_origin_x = gt_occupancy_grid_.info.origin.position.x;
        double gt_origin_y = gt_occupancy_grid_.info.origin.position.y;
        int gt_width = gt_occupancy_grid_.info.width;
        int gt_height = gt_occupancy_grid_.info.height;

        double curr_resolution = current_map_->info.resolution;
        double curr_origin_x = current_map_->info.origin.position.x;
        double curr_origin_y = current_map_->info.origin.position.y;
        int curr_width = current_map_->info.width;
        int curr_height = current_map_->info.height;

        for (int gy = 0; gy < gt_height; ++gy) {
            for (int gx = 0; gx < gt_width; ++gx) {
                int gt_idx = gy * gt_width + gx;
                int8_t gt_val = gt_occupancy_grid_.data[gt_idx];

                // GT'de unknown ise atla
                if (gt_val == -1) continue;

                bool gt_free = (gt_val < free_threshold_);
                bool gt_occupied = (gt_val >= occupied_threshold_);

                double wx = gt_origin_x + (gx + 0.5) * gt_resolution;
                double wy = gt_origin_y + (gy + 0.5) * gt_resolution;

                int cx = static_cast<int>((wx - curr_origin_x) / curr_resolution);
                int cy = static_cast<int>((wy - curr_origin_y) / curr_resolution);

                // Sınır dışı ise atla (henüz keşfedilmemiş sayılır)
                if (cx < 0 || cx >= curr_width || cy < 0 || cy >= curr_height) continue;

                int curr_idx = cy * curr_width + cx;
                int8_t curr_val = current_map_->data[curr_idx];

                // Anlık haritada unknown ise atla (henüz keşfedilmemiş)
                if (curr_val == -1) continue;

                bool curr_free = (curr_val < free_threshold_);
                bool curr_occupied = (curr_val >= occupied_threshold_);

                // Değerler eşleşiyor mu?
                if ((gt_free && curr_free) || (gt_occupied && curr_occupied)) {
                    matched++;
                } else {
                    mismatched++;
                }
            }
        }

        // Keşif yüzdesi = doğru eşleşen / GT toplam known
        // Yanlış yüzdesi = yanlış eşleşen / GT toplam known
        msg.exploration_2d_percentage = (gt_known_cells_ > 0) ?
            (static_cast<double>(matched) / gt_known_cells_) * 100.0 : 0.0;

        double error_percentage = (gt_known_cells_ > 0) ?
            (static_cast<double>(mismatched) / gt_known_cells_) * 100.0 : 0.0;

        // Debug log
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "2D: Keşif=%.1f%% (%d/%d) | Yanlış=%.1f%% (%d/%d)",
            msg.exploration_2d_percentage, matched, gt_known_cells_,
            error_percentage, mismatched, gt_known_cells_);
    }

    void calculateOctomapMetrics(exploration_metrics::msg::ExplorationMetrics& msg)
    {
        if (!current_octomap_ || !gt_octomap_ || gt_known_voxels_ == 0) return;

        int matched = 0;      // GT ile aynı konumda VE aynı değerde
        int mismatched = 0;   // GT ile aynı konumda AMA farklı değerde

        for (auto it = gt_octomap_->begin_leafs(); it != gt_octomap_->end_leafs(); ++it) {
            octomap::point3d coord = it.getCoordinate();

            octomap::OcTreeNode* curr_node = current_octomap_->search(coord);
            if (curr_node) {
                bool gt_occupied = gt_octomap_->isNodeOccupied(*it);
                bool curr_occupied = current_octomap_->isNodeOccupied(curr_node);

                if (gt_occupied == curr_occupied) {
                    matched++;
                } else {
                    mismatched++;
                }
            }
            // curr_node yoksa henüz keşfedilmemiş, atla
        }

        // Keşif yüzdesi = doğru eşleşen / GT toplam known
        // Yanlış yüzdesi = yanlış eşleşen / GT toplam known
        msg.exploration_3d_percentage = (gt_known_voxels_ > 0) ?
            (static_cast<double>(matched) / gt_known_voxels_) * 100.0 : 0.0;

        double error_percentage = (gt_known_voxels_ > 0) ?
            (static_cast<double>(mismatched) / gt_known_voxels_) * 100.0 : 0.0;

        // Debug log
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "3D: Keşif=%.1f%% (%d/%d) | Yanlış=%.1f%% (%d/%d)",
            msg.exploration_3d_percentage, matched, gt_known_voxels_,
            error_percentage, mismatched, gt_known_voxels_);
    }

    void publishVisualizationMarkers(const exploration_metrics::msg::ExplorationMetrics& msg)
    {
        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = now();
        text_marker.ns = "exploration_metrics";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = 0.0;
        text_marker.pose.position.y = 0.0;
        text_marker.pose.position.z = 3.0;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        char text_buf[512];
        snprintf(text_buf, sizeof(text_buf),
                 "[%s]\n2D: %.1f%% | 3D: %.1f%%\nPath: %.1fm | Vel: %.2fm/s",
                 map_name_.c_str(),
                 msg.exploration_2d_percentage,
                 msg.exploration_3d_percentage,
                 telemetry_received_ ? latest_telemetry_.total_path_traveled : 0.0,
                 telemetry_received_ ? latest_telemetry_.velocity_horizontal : 0.0);
        text_marker.text = text_buf;

        markers.markers.push_back(text_marker);

        // Progress bar background
        visualization_msgs::msg::Marker bar_bg;
        bar_bg.header.frame_id = "map";
        bar_bg.header.stamp = now();
        bar_bg.ns = "exploration_metrics";
        bar_bg.id = 1;
        bar_bg.type = visualization_msgs::msg::Marker::CUBE;
        bar_bg.action = visualization_msgs::msg::Marker::ADD;
        bar_bg.pose.position.x = 0.0;
        bar_bg.pose.position.y = 0.0;
        bar_bg.pose.position.z = 2.5;
        bar_bg.scale.x = 2.0;
        bar_bg.scale.y = 0.2;
        bar_bg.scale.z = 0.1;
        bar_bg.color.r = 0.3;
        bar_bg.color.g = 0.3;
        bar_bg.color.b = 0.3;
        bar_bg.color.a = 0.8;
        markers.markers.push_back(bar_bg);

        // Progress bar foreground
        visualization_msgs::msg::Marker bar_fg;
        bar_fg.header.frame_id = "map";
        bar_fg.header.stamp = now();
        bar_fg.ns = "exploration_metrics";
        bar_fg.id = 2;
        bar_fg.type = visualization_msgs::msg::Marker::CUBE;
        bar_fg.action = visualization_msgs::msg::Marker::ADD;
        double progress = msg.exploration_percentage / 100.0;
        double bar_width = 2.0 * progress;
        bar_fg.pose.position.x = -1.0 + bar_width / 2.0;
        bar_fg.pose.position.y = 0.0;
        bar_fg.pose.position.z = 2.5;
        bar_fg.scale.x = bar_width > 0.01 ? bar_width : 0.01;
        bar_fg.scale.y = 0.2;
        bar_fg.scale.z = 0.12;
        bar_fg.color.r = 0.2;
        bar_fg.color.g = 0.8;
        bar_fg.color.b = 0.2;
        bar_fg.color.a = 1.0;
        markers.markers.push_back(bar_fg);

        markers_pub_->publish(markers);
    }

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        start_time_wall_ = std::chrono::steady_clock::now();
        tick_counter_ = 0;
        exploration_active_ = false;
        last_exploration_percentage_ = -1.0;
        last_elapsed_time_ = 0.0;
        avg_exploration_rate_ = -1.0;

        if (csv_file_.is_open()) {
            csv_file_.close();
        }

        response->success = true;
        response->message = "Exploration metrics reset";
        RCLCPP_INFO(get_logger(), "Metrics reset");
    }

    void newLogCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Close existing log file if open
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
            RCLCPP_INFO(get_logger(), "Closed previous log: %s", current_log_file_.c_str());
        }

        // Reset timing for new session
        exploration_start_time_ = std::chrono::steady_clock::now();
        exploration_active_ = true;
        last_exploration_percentage_ = -1.0;
        avg_exploration_rate_ = -1.0;
        telemetry_log_counter_ = 0;

        // Create new log file
        initCsvFile();

        response->success = true;
        response->message = "New log started: " + current_log_file_;
        RCLCPP_INFO(get_logger(), "=== New log started: %s ===", current_log_file_.c_str());
    }

    // Parameters
    std::string comparison_mode_;
    std::string map_name_;
    std::string base_path_;
    double publish_rate_;
    bool logging_enabled_;
    int occupied_threshold_;
    int free_threshold_;
    double position_tolerance_;
    std::string map_topic_;
    std::string octomap_topic_;
    double ema_alpha_;
    double telemetry_log_rate_;

    // Paths
    std::string gt_octomap_path_;
    std::string gt_occupancy_grid_path_;
    std::string log_dir_;
    std::string current_log_file_;

    // Ground truth data
    std::shared_ptr<octomap::OcTree> gt_octomap_;
    nav_msgs::msg::OccupancyGrid gt_occupancy_grid_;
    bool gt_octomap_loaded_ = false;
    bool gt_occupancy_grid_loaded_ = false;
    int gt_known_voxels_ = 0;
    int gt_known_cells_ = 0;

    // Current data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    std::shared_ptr<octomap::OcTree> current_octomap_;
    bool map_received_ = false;
    bool octomap_received_ = false;

    // Telemetry data
    exploration_planner::msg::TelemetryStatus latest_telemetry_;
    bool telemetry_received_ = false;
    int telemetry_log_counter_ = 0;

    // Exploration state
    uint8_t current_exploration_state_ = 0;  // IDLE
    bool exploration_active_ = false;

    // Timing
    std::chrono::steady_clock::time_point start_time_wall_;
    std::chrono::steady_clock::time_point exploration_start_time_;
    uint64_t tick_counter_ = 0;
    double last_elapsed_time_ = 0.0;
    double last_exploration_percentage_ = -1.0;
    double avg_exploration_rate_ = -1.0;

    // Cached values for CSV logging
    double last_exploration_pct_ = 0.0;
    double last_2d_pct_ = 0.0;
    double last_3d_pct_ = 0.0;
    double last_exploration_rate_ = 0.0;

    // ROS interfaces
    rclcpp::Publisher<exploration_metrics::msg::ExplorationMetrics>::SharedPtr metrics_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<exploration_planner::msg::TelemetryStatus>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr exploration_status_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr new_log_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // CSV logging
    std::ofstream csv_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorationMetricsNode>());
    rclcpp::shutdown();
    return 0;
}
