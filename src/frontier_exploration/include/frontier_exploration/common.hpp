#ifndef FRONTIER_EXPLORATION__COMMON_HPP_
#define FRONTIER_EXPLORATION__COMMON_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>
#include <vector>

namespace frontier_exploration
{

// ============ Utility Functions ============

inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double angleDiff(double a1, double a2) {
  double diff = std::abs(normalizeAngle(a1 - a2));
  return std::min(diff, 2.0 * M_PI - diff);
}

inline int getIndex(int x, int y, int width) {
  return y * width + x;
}

inline geometry_msgs::msg::Point gridToWorld(int x, int y, 
                                              const nav_msgs::msg::OccupancyGrid& map) {
  geometry_msgs::msg::Point point;
  point.x = map.info.origin.position.x + (x + 0.5) * map.info.resolution;
  point.y = map.info.origin.position.y + (y + 0.5) * map.info.resolution;
  point.z = 0.0;
  return point;
}

inline std::pair<int, int> worldToGrid(double wx, double wy,
                                        const nav_msgs::msg::OccupancyGrid& map) {
  int gx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int gy = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return {gx, gy};
}

inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a = 0.8f) {
  std_msgs::msg::ColorRGBA color;
  color.r = r; color.g = g; color.b = b; color.a = a;
  return color;
}

// Color palette for clusters
inline std::vector<std_msgs::msg::ColorRGBA> getColorPalette() {
  return {
    createColor(1.0f, 0.0f, 0.0f),
    createColor(0.0f, 1.0f, 0.0f),
    createColor(0.0f, 0.0f, 1.0f),
    createColor(1.0f, 1.0f, 0.0f),
    createColor(1.0f, 0.0f, 1.0f),
    createColor(0.0f, 1.0f, 1.0f),
    createColor(1.0f, 0.5f, 0.0f),
    createColor(0.5f, 0.0f, 1.0f),
    createColor(0.0f, 1.0f, 0.5f),
    createColor(1.0f, 0.0f, 0.5f),
    createColor(0.5f, 1.0f, 0.0f),
    createColor(0.0f, 0.5f, 1.0f),
  };
}

// ============ Occupancy Check Functions ============

inline bool isFree(int8_t value, int8_t free_threshold = 25) {
  return value >= 0 && value < free_threshold;
}

inline bool isUnknown(int8_t value) {
  return value == -1;
}

inline bool isOccupied(int8_t value, int8_t occupied_threshold = 65) {
  return value >= occupied_threshold;
}

// ============ 4-Connected Neighbors ============

const std::vector<std::pair<int, int>> NEIGHBORS_4 = {
  {1, 0}, {-1, 0}, {0, 1}, {0, -1}
};

// ============ 8-Connected Neighbors ============

const std::vector<std::pair<int, int>> NEIGHBORS_8 = {
  {-1, -1}, {0, -1}, {1, -1},
  {-1,  0},          {1,  0},
  {-1,  1}, {0,  1}, {1,  1}
};

}  // namespace frontier_exploration

#endif  // FRONTIER_EXPLORATION__COMMON_HPP_