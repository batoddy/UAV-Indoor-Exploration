/**
 * Costmap Generator Node
 * 
 * Input:  /octomap_binary (octomap_msgs/Octomap)
 * Output: /exploration/costmap (nav_msgs/OccupancyGrid)
 * 
 * Converts 3D OctoMap to 2D costmap with:
 * - Height filtering (only consider obstacles between min_height and max_height)
 * - Robot footprint consideration (robot_width x robot_length)
 * - Lethal zone (robot can't fit) + inflation zone (high cost near obstacles)
 * - UNKNOWN AREA PENALTY: Bilinmeyen alanlara yüksek maliyet ver
 *   → Drone önce kamerayla görsün, sonra girsin
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <cmath>
#include <queue>
#include <set>

class CostmapGeneratorNode : public rclcpp::Node
{
public:
  CostmapGeneratorNode() : Node("costmap_generator")
  {
    // Height filter parameters
    declare_parameter("min_height", 0.5);
    declare_parameter("max_height", 3.0);
    
    // Robot footprint
    declare_parameter("robot_width", 0.5);
    declare_parameter("robot_length", 0.5);
    
    // Costmap parameters
    declare_parameter("costmap_resolution", 0.1);
    declare_parameter("inflation_radius", 1.0);
    declare_parameter("costmap_frame", "map");
    
    // Cost values for obstacles
    declare_parameter("lethal_cost", 100);
    declare_parameter("inscribed_cost", 99);
    declare_parameter("decay_factor", 2.0);
    
    // === UNKNOWN AREA PARAMETERS ===
    declare_parameter("unknown_cost", 60);              // Bilinmeyen alan maliyeti (0-100)
    declare_parameter("unknown_inflation_radius", 0.3); // Bilinmeyen alan şişirme
    declare_parameter("treat_unknown_as_obstacle", true); // Bilinmeyeni engel gibi şişir
    
    // Map size
    declare_parameter("map_size_x", 100.0);
    declare_parameter("map_size_y", 100.0);
    
    min_height_ = get_parameter("min_height").as_double();
    max_height_ = get_parameter("max_height").as_double();
    robot_width_ = get_parameter("robot_width").as_double();
    robot_length_ = get_parameter("robot_length").as_double();
    resolution_ = get_parameter("costmap_resolution").as_double();
    inflation_radius_ = get_parameter("inflation_radius").as_double();
    costmap_frame_ = get_parameter("costmap_frame").as_string();
    lethal_cost_ = get_parameter("lethal_cost").as_int();
    inscribed_cost_ = get_parameter("inscribed_cost").as_int();
    decay_factor_ = get_parameter("decay_factor").as_double();
    unknown_cost_ = get_parameter("unknown_cost").as_int();
    unknown_inflation_radius_ = get_parameter("unknown_inflation_radius").as_double();
    treat_unknown_as_obstacle_ = get_parameter("treat_unknown_as_obstacle").as_bool();
    map_size_x_ = get_parameter("map_size_x").as_double();
    map_size_y_ = get_parameter("map_size_y").as_double();
    
    // Calculate inscribed radius
    inscribed_radius_ = std::max(robot_width_, robot_length_) / 2.0;
    
    // Subscriber
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", 10,
      std::bind(&CostmapGeneratorNode::octomapCallback, this, std::placeholders::_1));
    
    // Publisher
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/exploration/costmap", 10);
    
    RCLCPP_INFO(get_logger(), "Costmap Generator initialized");
    RCLCPP_INFO(get_logger(), "  Height filter: %.1f - %.1f m", min_height_, max_height_);
    RCLCPP_INFO(get_logger(), "  Robot footprint: %.2f x %.2f m", robot_width_, robot_length_);
    RCLCPP_INFO(get_logger(), "  Inscribed radius: %.2f m", inscribed_radius_);
    RCLCPP_INFO(get_logger(), "  Inflation radius: %.2f m", inflation_radius_);
    RCLCPP_INFO(get_logger(), "  Unknown cost: %d (inflation: %.2f m)", 
                unknown_cost_, unknown_inflation_radius_);
  }

private:
  // Cell state enum
  static constexpr int UNKNOWN_CELL = 0;
  static constexpr int FREE_CELL = 1;
  static constexpr int OCCUPIED_CELL = 2;

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    if (!abstract_tree) {
      RCLCPP_WARN(get_logger(), "Failed to convert OctoMap message");
      return;
    }
    
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
    if (!octree) {
      RCLCPP_WARN(get_logger(), "Failed to cast to OcTree");
      delete abstract_tree;
      return;
    }
    
    // Get OctoMap bounds
    double x_min, y_min, z_min, x_max, y_max, z_max;
    octree->getMetricMin(x_min, y_min, z_min);
    octree->getMetricMax(x_max, y_max, z_max);
    
    // Add padding
    x_min -= inflation_radius_;
    y_min -= inflation_radius_;
    x_max += inflation_radius_;
    y_max += inflation_radius_;
    
    // Clamp to max size
    x_min = std::max(x_min, -map_size_x_ / 2.0);
    x_max = std::min(x_max, map_size_x_ / 2.0);
    y_min = std::max(y_min, -map_size_y_ / 2.0);
    y_max = std::min(y_max, map_size_y_ / 2.0);
    
    // Create costmap
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.stamp = msg->header.stamp;
    costmap.header.frame_id = costmap_frame_;
    
    costmap.info.resolution = resolution_;
    costmap.info.width = static_cast<uint32_t>((x_max - x_min) / resolution_) + 1;
    costmap.info.height = static_cast<uint32_t>((y_max - y_min) / resolution_) + 1;
    costmap.info.origin.position.x = x_min;
    costmap.info.origin.position.y = y_min;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 1.0;
    
    int width = costmap.info.width;
    int height = costmap.info.height;
    
    // Track cell states separately
    std::vector<int> cell_states(width * height, UNKNOWN_CELL);
    
    // Initialize costmap with unknown cost
    costmap.data.resize(width * height, unknown_cost_);
    
    // Collect obstacle and free cells
    std::vector<std::pair<int, int>> obstacle_cells;
    
    // Iterate through OctoMap leaves
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs();
         it != octree->end_leafs(); ++it)
    {
      double z = it.getZ();
      
      // Height filter
      if (z < min_height_ || z > max_height_) {
        continue;
      }
      
      double x = it.getX();
      double y = it.getY();
      double node_size = it.getSize();
      
      // For large voxels, mark multiple cells
      int cells_per_voxel = std::max(1, static_cast<int>(node_size / resolution_));
      
      for (int vi = 0; vi < cells_per_voxel; ++vi) {
        for (int vj = 0; vj < cells_per_voxel; ++vj) {
          double vx = x - node_size/2 + (vi + 0.5) * resolution_;
          double vy = y - node_size/2 + (vj + 0.5) * resolution_;
          
          int gx = static_cast<int>((vx - x_min) / resolution_);
          int gy = static_cast<int>((vy - y_min) / resolution_);
          
          if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
            continue;
          }
          
          int idx = gy * width + gx;
          
          if (octree->isNodeOccupied(*it)) {
            // Occupied
            cell_states[idx] = OCCUPIED_CELL;
            costmap.data[idx] = lethal_cost_;
            obstacle_cells.push_back({gx, gy});
          } else {
            // Free - only update if not already occupied
            if (cell_states[idx] != OCCUPIED_CELL) {
              cell_states[idx] = FREE_CELL;
              costmap.data[idx] = 0;  // Free = 0 cost
            }
          }
        }
      }
    }
    
    // Inflate obstacles
    inflateObstacles(costmap, obstacle_cells, cell_states);
    
    // Inflate unknown areas if enabled
    if (treat_unknown_as_obstacle_) {
      inflateUnknown(costmap, cell_states, width, height);
    }
    
    costmap_pub_->publish(costmap);
    
    // Count cells for debug
    int unknown_count = 0, free_count = 0, obstacle_count = 0;
    for (int i = 0; i < width * height; ++i) {
      if (cell_states[i] == UNKNOWN_CELL) unknown_count++;
      else if (cell_states[i] == FREE_CELL) free_count++;
      else obstacle_count++;
    }
    
    RCLCPP_DEBUG(get_logger(), "Costmap: %dx%d, Free:%d, Obstacle:%d, Unknown:%d",
                 width, height, free_count, obstacle_count, unknown_count);
    
    delete octree;
  }
  
  void inflateObstacles(nav_msgs::msg::OccupancyGrid& costmap,
                        const std::vector<std::pair<int, int>>& obstacles,
                        const std::vector<int>& /* cell_states */)  // Reserved for future use
  {
    int width = costmap.info.width;
    int height = costmap.info.height;
    
    int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    
    for (const auto& [ox, oy] : obstacles) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          int nx = ox + dx;
          int ny = oy + dy;
          
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          
          int nidx = ny * width + nx;
          
          // Skip if already lethal
          if (costmap.data[nidx] == lethal_cost_) continue;
          
          double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
          
          int8_t cost = 0;
          
          if (dist <= inscribed_radius_) {
            cost = inscribed_cost_;
          } else if (dist <= inflation_radius_) {
            double ratio = (dist - inscribed_radius_) / (inflation_radius_ - inscribed_radius_);
            cost = static_cast<int8_t>(inscribed_cost_ * std::exp(-decay_factor_ * ratio));
            cost = std::max(static_cast<int8_t>(1), cost);
          }
          
          // Keep higher cost (obstacle inflation > unknown)
          if (cost > costmap.data[nidx]) {
            costmap.data[nidx] = cost;
          }
        }
      }
    }
  }
  
  void inflateUnknown(nav_msgs::msg::OccupancyGrid& costmap,
                      const std::vector<int>& cell_states,
                      int width, int height)
  {
    // Create a copy to avoid modifying while iterating
    std::vector<int8_t> original_data = costmap.data;
    
    int unknown_inflation_cells = static_cast<int>(std::ceil(unknown_inflation_radius_ / resolution_));
    
    // Find unknown cells and inflate them
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        
        // If this is an unknown cell
        if (cell_states[idx] == UNKNOWN_CELL) {
          // Inflate around it
          for (int dy = -unknown_inflation_cells; dy <= unknown_inflation_cells; ++dy) {
            for (int dx = -unknown_inflation_cells; dx <= unknown_inflation_cells; ++dx) {
              int nx = x + dx;
              int ny = y + dy;
              
              if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
              
              int nidx = ny * width + nx;
              
              // Don't overwrite obstacles or higher costs
              if (original_data[nidx] >= unknown_cost_) continue;
              
              double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
              
              if (dist <= unknown_inflation_radius_) {
                // Decay unknown cost with distance
                double ratio = dist / unknown_inflation_radius_;
                int8_t cost = static_cast<int8_t>(unknown_cost_ * (1.0 - ratio * 0.5));
                
                if (cost > costmap.data[nidx]) {
                  costmap.data[nidx] = cost;
                }
              }
            }
          }
        }
      }
    }
  }
  
  // Parameters
  double min_height_;
  double max_height_;
  double robot_width_;
  double robot_length_;
  double inscribed_radius_;
  double resolution_;
  double inflation_radius_;
  std::string costmap_frame_;
  int lethal_cost_;
  int inscribed_cost_;
  double decay_factor_;
  int unknown_cost_;
  double unknown_inflation_radius_;
  bool treat_unknown_as_obstacle_;
  double map_size_x_;
  double map_size_y_;
  
  // ROS
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
