cmake_minimum_required(VERSION 3.8)
project(frontier_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Frontier detector node
add_executable(frontier_detector_node
  src/frontier_detector_node.cpp
  src/frontier_detector.cpp
)
ament_target_dependencies(frontier_detector_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install
install(TARGETS
  frontier_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.yaml"
  PATTERN "*.rviz"
)

ament_package()
