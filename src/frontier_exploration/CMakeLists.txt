cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

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
find_package(rosidl_default_generators REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Viewpoint.msg"
  "msg/ClusterConnection.msg"
  "msg/FrontierCluster.msg"
  "msg/FrontierArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(include ${OCTOMAP_INCLUDE_DIRS})

# Common dependencies
set(COMMON_DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ============ Node 1: Frontier Detector ============
add_executable(frontier_detector_node src/frontier_detector_node.cpp)
ament_target_dependencies(frontier_detector_node ${COMMON_DEPS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(frontier_detector_node "${cpp_typesupport_target}")

# ============ Node 2: Viewpoint Generator (Feature-rich + Occlusion-aware + Stabilized) ============
add_executable(viewpoint_generator_node src/viewpoint_generator_node.cpp)
ament_target_dependencies(viewpoint_generator_node ${COMMON_DEPS} octomap_msgs)
target_link_libraries(viewpoint_generator_node "${cpp_typesupport_target}" ${OCTOMAP_LIBRARIES})

# ============ Node 3: Frontier Visualizer ============
add_executable(frontier_visualizer_node src/frontier_visualizer_node.cpp)
ament_target_dependencies(frontier_visualizer_node ${COMMON_DEPS})
target_link_libraries(frontier_visualizer_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  frontier_detector_node
  viewpoint_generator_node
  frontier_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()