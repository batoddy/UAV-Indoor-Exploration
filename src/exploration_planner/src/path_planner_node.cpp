/**
 * Path Planner Node
 * 
 * Input:  /exploration/costmap (nav_msgs/OccupancyGrid)
 *         /exploration/global_tour (exploration_planner/ExplorationStatus)
 *         /odom (current position)
 * Output: /exploration/planned_path (nav_msgs/Path)
 * 
 * Supports two algorithms:
 * - A* (optimal, slower for large maps)
 * - RRT (fast, suboptimal but good for complex environments)
 * 
 * Configurable via 'planner_type' parameter: "astar" or "rrt"
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "exploration_planner/msg/exploration_status.hpp"

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>

struct GridCell {
  int x, y;
  
  bool operator==(const GridCell& other) const {
    return x == other.x && y == other.y;
  }
};

struct GridCellHash {
  std::size_t operator()(const GridCell& cell) const {
    return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 16);
  }
};

struct AStarNode {
  GridCell cell;
  double g_cost;  // Cost from start
  double f_cost;  // g + heuristic
  GridCell parent;
  
  bool operator>(const AStarNode& other) const {
    return f_cost > other.f_cost;
  }
};

struct RRTNode {
  double x, y;
  int parent_idx;
};

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode() : Node("path_planner"), rng_(std::random_device{}())
  {
    // Planner selection
    declare_parameter("planner_type", "astar");  // "astar" or "rrt"
    
    // Common parameters
    declare_parameter("obstacle_threshold", 50);  // Cost value considered obstacle
    declare_parameter("path_simplify", true);     // Simplify path with line-of-sight
    
    // A* parameters
    declare_parameter("astar_allow_diagonal", true);
    
    // RRT parameters
    declare_parameter("rrt_max_iterations", 5000);
    declare_parameter("rrt_step_size", 0.5);      // [m]
    declare_parameter("rrt_goal_bias", 0.1);      // Probability to sample goal
    declare_parameter("rrt_goal_threshold", 0.5); // [m] Distance to consider goal reached
    
    planner_type_ = get_parameter("planner_type").as_string();
    obstacle_threshold_ = get_parameter("obstacle_threshold").as_int();
    path_simplify_ = get_parameter("path_simplify").as_bool();
    astar_allow_diagonal_ = get_parameter("astar_allow_diagonal").as_bool();
    rrt_max_iterations_ = get_parameter("rrt_max_iterations").as_int();
    rrt_step_size_ = get_parameter("rrt_step_size").as_double();
    rrt_goal_bias_ = get_parameter("rrt_goal_bias").as_double();
    rrt_goal_threshold_ = get_parameter("rrt_goal_threshold").as_double();
    
    // Subscribers
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/exploration/costmap", 10,
      std::bind(&PathPlannerNode::costmapCallback, this, std::placeholders::_1));
    
    tour_sub_ = create_subscription<exploration_planner::msg::ExplorationStatus>(
      "/exploration/global_tour", 10,
      std::bind(&PathPlannerNode::tourCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/planned_path", 10);
    
    RCLCPP_INFO(get_logger(), "Path Planner initialized");
    RCLCPP_INFO(get_logger(), "  Planner type: %s", planner_type_.c_str());
    RCLCPP_INFO(get_logger(), "  Obstacle threshold: %d", obstacle_threshold_);
  }

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    current_costmap_ = msg;
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    have_pose_ = true;
  }
  
  void tourCallback(const exploration_planner::msg::ExplorationStatus::SharedPtr msg)
  {
    if (!have_pose_ || !current_costmap_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                           "Waiting for pose and costmap...");
      return;
    }
    
    if (msg->waypoints.empty()) {
      return;
    }
    
    // Plan path from current position to first waypoint
    geometry_msgs::msg::Point start;
    start.x = current_pose_.position.x;
    start.y = current_pose_.position.y;
    
    geometry_msgs::msg::Point goal;
    goal.x = msg->waypoints[0].pose.position.x;
    goal.y = msg->waypoints[0].pose.position.y;
    
    // IMPORTANT: Save target orientation from global_tour_planner
    // This contains the yaw where the camera should look at the frontier
    target_orientation_ = msg->waypoints[0].pose.orientation;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = current_costmap_->header.frame_id;
    
    bool success = false;
    
    if (planner_type_ == "astar") {
      success = planAStar(start, goal, path);
    } else if (planner_type_ == "rrt") {
      success = planRRT(start, goal, path);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown planner type: %s", planner_type_.c_str());
      return;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double planning_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    if (success) {
      // Simplify path if enabled
      if (path_simplify_ && path.poses.size() > 2) {
        simplifyPath(path);
      }
      
      // Set the FINAL waypoint's orientation to target orientation
      // This preserves the yaw from global_tour_planner
      if (!path.poses.empty()) {
        path.poses.back().pose.orientation = target_orientation_;
      }
      
      path_pub_->publish(path);
      
      RCLCPP_INFO(get_logger(), "[%s] Path found: %zu points, %.1f ms",
                  planner_type_.c_str(), path.poses.size(), planning_time);
    } else {
      RCLCPP_WARN(get_logger(), "[%s] No path found! (%.1f ms)",
                  planner_type_.c_str(), planning_time);
      
      // Publish empty path to signal failure
      path_pub_->publish(path);
    }
  }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // A* ALGORITHM
  // ═══════════════════════════════════════════════════════════════════════════
  
  bool planAStar(const geometry_msgs::msg::Point& start,
                 const geometry_msgs::msg::Point& goal,
                 nav_msgs::msg::Path& path)
  {
    // Convert to grid coordinates
    GridCell start_cell = worldToGrid(start.x, start.y);
    GridCell goal_cell = worldToGrid(goal.x, goal.y);
    
    // Check if start/goal are valid
    if (!isValidCell(start_cell) || !isValidCell(goal_cell)) {
      RCLCPP_WARN(get_logger(), "A*: Start or goal outside map");
      return false;
    }
    
    if (isObstacle(start_cell)) {
      RCLCPP_WARN(get_logger(), "A*: Start position is in obstacle");
      return false;
    }
    
    if (isObstacle(goal_cell)) {
      RCLCPP_WARN(get_logger(), "A*: Goal position is in obstacle");
      return false;
    }
    
    // Priority queue (min-heap by f_cost)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_set<GridCell, GridCellHash> closed_set;
    std::unordered_map<GridCell, AStarNode, GridCellHash> all_nodes;
    
    // Initialize start node
    AStarNode start_node;
    start_node.cell = start_cell;
    start_node.g_cost = 0;
    start_node.f_cost = heuristic(start_cell, goal_cell);
    start_node.parent = start_cell;
    
    open_set.push(start_node);
    all_nodes[start_cell] = start_node;
    
    // Direction vectors (8-connected if diagonal allowed, else 4-connected)
    std::vector<std::pair<int, int>> directions;
    if (astar_allow_diagonal_) {
      directions = {{1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}};
    } else {
      directions = {{1,0}, {-1,0}, {0,1}, {0,-1}};
    }
    
    while (!open_set.empty()) {
      AStarNode current = open_set.top();
      open_set.pop();
      
      // Skip if already processed
      if (closed_set.count(current.cell)) continue;
      closed_set.insert(current.cell);
      
      // Goal reached?
      if (current.cell == goal_cell) {
        // Reconstruct path
        reconstructPath(all_nodes, goal_cell, start_cell, path);
        return true;
      }
      
      // Expand neighbors
      for (const auto& dir : directions) {
        GridCell neighbor;
        neighbor.x = current.cell.x + dir.first;
        neighbor.y = current.cell.y + dir.second;
        
        if (!isValidCell(neighbor) || closed_set.count(neighbor) || isObstacle(neighbor)) {
          continue;
        }
        
        // Calculate cost
        double move_cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0;
        move_cost *= current_costmap_->info.resolution;
        
        // Add cost from costmap (prefer lower cost cells)
        int cell_cost = getCellCost(neighbor);
        if (cell_cost > 0 && cell_cost < obstacle_threshold_) {
          move_cost *= (1.0 + cell_cost / 100.0);  // Inflate cost near obstacles
        }
        
        double new_g = current.g_cost + move_cost;
        
        // Check if this is a better path
        auto it = all_nodes.find(neighbor);
        if (it == all_nodes.end() || new_g < it->second.g_cost) {
          AStarNode neighbor_node;
          neighbor_node.cell = neighbor;
          neighbor_node.g_cost = new_g;
          neighbor_node.f_cost = new_g + heuristic(neighbor, goal_cell);
          neighbor_node.parent = current.cell;
          
          open_set.push(neighbor_node);
          all_nodes[neighbor] = neighbor_node;
        }
      }
    }
    
    return false;  // No path found
  }
  
  double heuristic(const GridCell& a, const GridCell& b)
  {
    // Euclidean distance
    double dx = (a.x - b.x) * current_costmap_->info.resolution;
    double dy = (a.y - b.y) * current_costmap_->info.resolution;
    return std::sqrt(dx*dx + dy*dy);
  }
  
  void reconstructPath(const std::unordered_map<GridCell, AStarNode, GridCellHash>& nodes,
                       const GridCell& goal, const GridCell& start,
                       nav_msgs::msg::Path& path)
  {
    std::vector<GridCell> cells;
    GridCell current = goal;
    
    while (!(current == start)) {
      cells.push_back(current);
      current = nodes.at(current).parent;
    }
    cells.push_back(start);
    
    // Reverse to get start-to-goal order
    std::reverse(cells.begin(), cells.end());
    
    // Convert to world coordinates
    for (const auto& cell : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      auto [wx, wy] = gridToWorld(cell.x, cell.y);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // RRT ALGORITHM
  // ═══════════════════════════════════════════════════════════════════════════
  
  bool planRRT(const geometry_msgs::msg::Point& start,
               const geometry_msgs::msg::Point& goal,
               nav_msgs::msg::Path& path)
  {
    // Check if start/goal are valid
    GridCell start_cell = worldToGrid(start.x, start.y);
    GridCell goal_cell = worldToGrid(goal.x, goal.y);
    
    if (!isValidCell(start_cell) || isObstacle(start_cell)) {
      RCLCPP_WARN(get_logger(), "RRT: Start position invalid");
      return false;
    }
    
    if (!isValidCell(goal_cell) || isObstacle(goal_cell)) {
      RCLCPP_WARN(get_logger(), "RRT: Goal position invalid");
      return false;
    }
    
    // Initialize tree with start node
    std::vector<RRTNode> tree;
    RRTNode start_node;
    start_node.x = start.x;
    start_node.y = start.y;
    start_node.parent_idx = -1;
    tree.push_back(start_node);
    
    // Sampling bounds
    double x_min = current_costmap_->info.origin.position.x;
    double y_min = current_costmap_->info.origin.position.y;
    double x_max = x_min + current_costmap_->info.width * current_costmap_->info.resolution;
    double y_max = y_min + current_costmap_->info.height * current_costmap_->info.resolution;
    
    std::uniform_real_distribution<double> x_dist(x_min, x_max);
    std::uniform_real_distribution<double> y_dist(y_min, y_max);
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    
    for (int iter = 0; iter < rrt_max_iterations_; ++iter) {
      // Sample random point (with goal bias)
      double sample_x, sample_y;
      if (bias_dist(rng_) < rrt_goal_bias_) {
        sample_x = goal.x;
        sample_y = goal.y;
      } else {
        sample_x = x_dist(rng_);
        sample_y = y_dist(rng_);
      }
      
      // Find nearest node in tree
      int nearest_idx = findNearestNode(tree, sample_x, sample_y);
      const RRTNode& nearest = tree[nearest_idx];
      
      // Steer towards sample
      double dx = sample_x - nearest.x;
      double dy = sample_y - nearest.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      
      double new_x, new_y;
      if (dist <= rrt_step_size_) {
        new_x = sample_x;
        new_y = sample_y;
      } else {
        new_x = nearest.x + (dx / dist) * rrt_step_size_;
        new_y = nearest.y + (dy / dist) * rrt_step_size_;
      }
      
      // Check if path to new node is collision-free
      if (!isPathFree(nearest.x, nearest.y, new_x, new_y)) {
        continue;
      }
      
      // Add new node
      RRTNode new_node;
      new_node.x = new_x;
      new_node.y = new_y;
      new_node.parent_idx = nearest_idx;
      tree.push_back(new_node);
      
      // Check if we reached the goal
      double goal_dist = std::sqrt(std::pow(new_x - goal.x, 2) + std::pow(new_y - goal.y, 2));
      if (goal_dist < rrt_goal_threshold_) {
        // Check path to goal is free
        if (isPathFree(new_x, new_y, goal.x, goal.y)) {
          // Add goal node
          RRTNode goal_node;
          goal_node.x = goal.x;
          goal_node.y = goal.y;
          goal_node.parent_idx = tree.size() - 1;
          tree.push_back(goal_node);
          
          // Reconstruct path
          reconstructRRTPath(tree, path);
          return true;
        }
      }
    }
    
    return false;  // Max iterations reached
  }
  
  int findNearestNode(const std::vector<RRTNode>& tree, double x, double y)
  {
    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < tree.size(); ++i) {
      double dist = std::sqrt(std::pow(tree[i].x - x, 2) + std::pow(tree[i].y - y, 2));
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = i;
      }
    }
    
    return nearest_idx;
  }
  
  bool isPathFree(double x1, double y1, double x2, double y2)
  {
    double dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    int steps = std::max(2, static_cast<int>(dist / (current_costmap_->info.resolution * 0.5)));
    
    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;
      double x = x1 + t * (x2 - x1);
      double y = y1 + t * (y2 - y1);
      
      GridCell cell = worldToGrid(x, y);
      if (!isValidCell(cell) || isObstacle(cell)) {
        return false;
      }
    }
    
    return true;
  }
  
  void reconstructRRTPath(const std::vector<RRTNode>& tree, nav_msgs::msg::Path& path)
  {
    std::vector<const RRTNode*> nodes;
    const RRTNode* current = &tree.back();
    
    while (current->parent_idx >= 0) {
      nodes.push_back(current);
      current = &tree[current->parent_idx];
    }
    nodes.push_back(current);  // Add start node
    
    // Reverse to get start-to-goal order
    std::reverse(nodes.begin(), nodes.end());
    
    for (const auto* node : nodes) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = node->x;
      pose.pose.position.y = node->y;
      pose.pose.position.z = 0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // PATH SIMPLIFICATION (Line-of-Sight)
  // ═══════════════════════════════════════════════════════════════════════════
  
  void simplifyPath(nav_msgs::msg::Path& path)
  {
    if (path.poses.size() < 3) return;
    
    std::vector<geometry_msgs::msg::PoseStamped> simplified;
    simplified.push_back(path.poses.front());
    
    size_t current = 0;
    
    while (current < path.poses.size() - 1) {
      // Find furthest visible point
      size_t furthest = current + 1;
      
      for (size_t i = current + 2; i < path.poses.size(); ++i) {
        if (isPathFree(path.poses[current].pose.position.x,
                       path.poses[current].pose.position.y,
                       path.poses[i].pose.position.x,
                       path.poses[i].pose.position.y)) {
          furthest = i;
        }
      }
      
      simplified.push_back(path.poses[furthest]);
      current = furthest;
    }
    
    RCLCPP_DEBUG(get_logger(), "Path simplified: %zu -> %zu points",
                 path.poses.size(), simplified.size());
    
    path.poses = simplified;
  }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // HELPER FUNCTIONS
  // ═══════════════════════════════════════════════════════════════════════════
  
  GridCell worldToGrid(double x, double y)
  {
    GridCell cell;
    cell.x = static_cast<int>((x - current_costmap_->info.origin.position.x) / 
                               current_costmap_->info.resolution);
    cell.y = static_cast<int>((y - current_costmap_->info.origin.position.y) / 
                               current_costmap_->info.resolution);
    return cell;
  }
  
  std::pair<double, double> gridToWorld(int gx, int gy)
  {
    double x = current_costmap_->info.origin.position.x + 
               (gx + 0.5) * current_costmap_->info.resolution;
    double y = current_costmap_->info.origin.position.y + 
               (gy + 0.5) * current_costmap_->info.resolution;
    return {x, y};
  }
  
  bool isValidCell(const GridCell& cell)
  {
    return cell.x >= 0 && cell.x < static_cast<int>(current_costmap_->info.width) &&
           cell.y >= 0 && cell.y < static_cast<int>(current_costmap_->info.height);
  }
  
  bool isObstacle(const GridCell& cell)
  {
    int idx = cell.y * current_costmap_->info.width + cell.x;
    return current_costmap_->data[idx] >= obstacle_threshold_;
  }
  
  int getCellCost(const GridCell& cell)
  {
    int idx = cell.y * current_costmap_->info.width + cell.x;
    return current_costmap_->data[idx];
  }
  
  // Parameters
  std::string planner_type_;
  int obstacle_threshold_;
  bool path_simplify_;
  bool astar_allow_diagonal_;
  int rrt_max_iterations_;
  double rrt_step_size_;
  double rrt_goal_bias_;
  double rrt_goal_threshold_;
  
  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Quaternion target_orientation_;  // Target yaw from global_tour
  bool have_pose_ = false;
  
  // RNG for RRT
  std::mt19937 rng_;
  
  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<exploration_planner::msg::ExplorationStatus>::SharedPtr tour_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
