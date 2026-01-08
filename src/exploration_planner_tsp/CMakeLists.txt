cmake_minimum_required(VERSION 3.8)
project(exploration_planner_tsp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
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

# ============ Node 1: Global Tour Planner (TSP) ============
add_executable(global_tour_planner_node src/global_tour_planner_node.cpp)
ament_target_dependencies(global_tour_planner_node ${COMMON_DEPS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(global_tour_planner_node "${cpp_typesupport_target}")

# ============ Node 2: Local Viewpoint Refiner ============
add_executable(local_refiner_node src/local_refiner_node.cpp)
ament_target_dependencies(local_refiner_node ${COMMON_DEPS})
target_link_libraries(local_refiner_node "${cpp_typesupport_target}")

# ============ Node 3: Trajectory Generator ============
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator_node ${COMMON_DEPS})
target_link_libraries(trajectory_generator_node "${cpp_typesupport_target}")

# ============ Node 4: Trajectory Follower (cmd_vel) ============
add_executable(trajectory_follower_node src/trajectory_follower_node.cpp)
ament_target_dependencies(trajectory_follower_node ${COMMON_DEPS})
target_link_libraries(trajectory_follower_node "${cpp_typesupport_target}")

# ============ Node 5: Exploration Visualizer ============
add_executable(exploration_visualizer_node src/exploration_visualizer_node.cpp)
ament_target_dependencies(exploration_visualizer_node ${COMMON_DEPS})
target_link_libraries(exploration_visualizer_node "${cpp_typesupport_target}")

install(TARGETS
  global_tour_planner_node
  local_refiner_node
  trajectory_generator_node
  trajectory_follower_node
  exploration_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()