cmake_minimum_required(VERSION 3.8)
project(exploration_metrics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(exploration_planner REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExplorationMetrics.msg"
  DEPENDENCIES std_msgs
)

# Common dependencies
set(COMMON_DEPS
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  visualization_msgs
  octomap_msgs
  tf2_ros
  tf2_geometry_msgs
  exploration_planner
)

# Get typesupport target for linking custom messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Exploration metrics node
add_executable(exploration_metrics_node src/exploration_metrics_node.cpp)
ament_target_dependencies(exploration_metrics_node ${COMMON_DEPS})
target_link_libraries(exploration_metrics_node
  ${OCTOMAP_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_include_directories(exploration_metrics_node PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
)

# Map saver node
add_executable(map_saver_node src/map_saver_node.cpp)
ament_target_dependencies(map_saver_node ${COMMON_DEPS})
target_link_libraries(map_saver_node
  ${OCTOMAP_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_include_directories(map_saver_node PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
)

# Install executables
install(TARGETS
  exploration_metrics_node
  map_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/plot_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
