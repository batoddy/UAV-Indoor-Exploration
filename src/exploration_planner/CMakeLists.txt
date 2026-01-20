cmake_minimum_required(VERSION 3.8)
project(exploration_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(frontier_exploration REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExplorationStatus.msg"
  "msg/Trajectory.msg"
  "msg/TelemetryStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

include_directories(include)

set(COMMON_DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  frontier_exploration
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ============ Global Tour Planner (Greedy Selector) ============
add_executable(global_tour_planner_node src/global_tour_planner_node.cpp)
ament_target_dependencies(global_tour_planner_node ${COMMON_DEPS})
target_link_libraries(global_tour_planner_node "${cpp_typesupport_target}")

# ============ Nav2 Path Planner ============
add_executable(nav2_path_planner_node src/nav2_path_planner_node.cpp)
ament_target_dependencies(nav2_path_planner_node rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs)
target_link_libraries(nav2_path_planner_node "${cpp_typesupport_target}")

# ============ Path Yaw Injector ============
add_executable(path_yaw_injector_node src/path_yaw_injector_node.cpp)
ament_target_dependencies(path_yaw_injector_node rclcpp nav_msgs geometry_msgs)

# ============ Trajectory Generator ============
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator_node ${COMMON_DEPS})
target_link_libraries(trajectory_generator_node "${cpp_typesupport_target}")

# ============ Trajectory Follower ============
add_executable(trajectory_follower_node src/trajectory_follower_node.cpp)
ament_target_dependencies(trajectory_follower_node ${COMMON_DEPS})
target_link_libraries(trajectory_follower_node "${cpp_typesupport_target}")

# ============ Exploration Visualizer ============
add_executable(exploration_visualizer_node src/exploration_visualizer_node.cpp)
ament_target_dependencies(exploration_visualizer_node ${COMMON_DEPS})
target_link_libraries(exploration_visualizer_node "${cpp_typesupport_target}")

# ============ Telemetry Node ============
add_executable(telemetry_node src/telemetry_node.cpp)
ament_target_dependencies(telemetry_node ${COMMON_DEPS})
target_link_libraries(telemetry_node "${cpp_typesupport_target}")

install(TARGETS
  global_tour_planner_node
  nav2_path_planner_node
  path_yaw_injector_node
  trajectory_generator_node
  trajectory_follower_node
  exploration_visualizer_node
  telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
