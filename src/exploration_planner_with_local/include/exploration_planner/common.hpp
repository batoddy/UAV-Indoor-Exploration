#ifndef EXPLORATION_PLANNER__COMMON_HPP_
#define EXPLORATION_PLANNER__COMMON_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <vector>
#include <limits>

namespace exploration_planner
{

// ============ Math Utilities ============

inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double angleDiff(double a1, double a2) {
  double diff = std::abs(normalizeAngle(a1 - a2));
  return std::min(diff, 2.0 * M_PI - diff);
}

inline double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double dz = b.z - a.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

inline double distance2D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  return std::sqrt(dx*dx + dy*dy);
}

inline double getYaw(const geometry_msgs::msg::Quaternion& q) {
  // Extract yaw from quaternion
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// ============ Time Lower Bound Calculation ============
// t_lb = max(path_length / v_max, yaw_change / yaw_rate_max)

inline double computeTimeLowerBound(
  const geometry_msgs::msg::Point& from_pos, double from_yaw,
  const geometry_msgs::msg::Point& to_pos, double to_yaw,
  double v_max, double yaw_rate_max)
{
  double dist = distance2D(from_pos, to_pos);
  double yaw_diff = angleDiff(from_yaw, to_yaw);
  
  double time_translation = dist / v_max;
  double time_rotation = yaw_diff / yaw_rate_max;
  
  return std::max(time_translation, time_rotation);
}

// ============ Motion Consistency Cost ============
// cc = acos((p_target - p_current) · v_current / (|...| * |...|))
// Penalizes large changes in flight direction

inline double computeMotionConsistencyCost(
  const geometry_msgs::msg::Point& current_pos,
  const geometry_msgs::msg::Point& current_vel,
  const geometry_msgs::msg::Point& target_pos)
{
  // Direction to target
  double dx = target_pos.x - current_pos.x;
  double dy = target_pos.y - current_pos.y;
  double dist = std::sqrt(dx*dx + dy*dy);
  
  if (dist < 0.01) return 0.0;
  
  // Current velocity magnitude
  double vel_mag = std::sqrt(current_vel.x*current_vel.x + current_vel.y*current_vel.y);
  
  if (vel_mag < 0.01) return 0.0;  // No velocity, no consistency cost
  
  // Dot product
  double dot = (dx * current_vel.x + dy * current_vel.y) / (dist * vel_mag);
  dot = std::max(-1.0, std::min(1.0, dot));  // Clamp for acos
  
  return std::acos(dot);  // Returns angle in radians [0, PI]
}

// ============ Simple Greedy TSP Solver ============
// Nearest neighbor heuristic - O(n²)

inline std::vector<size_t> solveGreedyTSP(
  const std::vector<std::vector<double>>& cost_matrix,
  size_t start_node = 0)
{
  size_t n = cost_matrix.size();
  if (n == 0) return {};
  if (n == 1) return {0};
  
  std::vector<size_t> tour;
  std::vector<bool> visited(n, false);
  
  size_t current = start_node;
  tour.push_back(current);
  visited[current] = true;
  
  for (size_t i = 1; i < n; ++i) {
    double best_cost = std::numeric_limits<double>::infinity();
    size_t best_next = 0;
    
    for (size_t j = 0; j < n; ++j) {
      if (!visited[j] && cost_matrix[current][j] < best_cost) {
        best_cost = cost_matrix[current][j];
        best_next = j;
      }
    }
    
    tour.push_back(best_next);
    visited[best_next] = true;
    current = best_next;
  }
  
  return tour;
}

// ============ 2-Opt TSP Improvement ============
// Local optimization to improve greedy solution

inline void improve2Opt(
  std::vector<size_t>& tour,
  const std::vector<std::vector<double>>& cost_matrix)
{
  if (tour.size() < 4) return;
  
  bool improved = true;
  while (improved) {
    improved = false;
    
    for (size_t i = 0; i < tour.size() - 2; ++i) {
      for (size_t j = i + 2; j < tour.size(); ++j) {
        // Calculate cost change if we reverse segment [i+1, j]
        size_t a = tour[i];
        size_t b = tour[i + 1];
        size_t c = tour[j];
        size_t d = (j + 1 < tour.size()) ? tour[j + 1] : tour[0];
        
        double old_cost = cost_matrix[a][b] + cost_matrix[c][d];
        double new_cost = cost_matrix[a][c] + cost_matrix[b][d];
        
        if (new_cost < old_cost - 1e-6) {
          // Reverse segment
          std::reverse(tour.begin() + i + 1, tour.begin() + j + 1);
          improved = true;
        }
      }
    }
  }
}

}  // namespace exploration_planner

#endif  // EXPLORATION_PLANNER__COMMON_HPP_