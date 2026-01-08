cmake_minimum_required(VERSION 3.8)
project(cmd_vel_to_px4)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Gazebo için
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# =========================
# ✅ PYTHON NODES
# =========================
install(PROGRAMS
  src/odom_converter.py
  src/teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# ✅ C++ NODE 1
# =========================
add_executable(cmd_vel_px4_bridge src/cmd_vel_px4_bridge.cpp)
ament_target_dependencies(cmd_vel_px4_bridge
  rclcpp
  geometry_msgs
  px4_msgs
)

# =========================
# ✅ C++ NODE 2 (DEPTH)
# =========================
add_executable(gz_depth_bridge src/gz_depth_bridge.cpp)
ament_target_dependencies(gz_depth_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(gz_depth_bridge
  gz-transport13::core
  gz-msgs10::core
)

# =========================
# ✅ INSTALL
# =========================
install(TARGETS
  cmd_vel_px4_bridge
  gz_depth_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()