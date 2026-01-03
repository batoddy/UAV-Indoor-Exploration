cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(frontier_interfaces REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
)

set(DEPS
  rclcpp
  frontier_interfaces
  octomap_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Node 1: Frontier Detector
add_executable(frontier_detector src/frontier_detector_node.cpp)
ament_target_dependencies(frontier_detector ${DEPS})
target_link_libraries(frontier_detector ${OCTOMAP_LIBRARIES})

# Node 2: Frontier Clusterer
add_executable(frontier_clusterer src/frontier_clusterer_node.cpp)
ament_target_dependencies(frontier_clusterer ${DEPS})

# Node 3: Cost Evaluator
add_executable(cost_evaluator src/cost_evaluator_node.cpp)
ament_target_dependencies(cost_evaluator ${DEPS})

# Node 4: Goal Selector
add_executable(goal_selector src/goal_selector_node.cpp)
ament_target_dependencies(goal_selector ${DEPS})

# Node 5: Global Planner (3D A*)
add_executable(global_planner src/global_planner_node.cpp)
ament_target_dependencies(global_planner ${DEPS})
target_link_libraries(global_planner ${OCTOMAP_LIBRARIES})

# Node 6: Local Planner
add_executable(local_planner src/local_planner_node.cpp)
ament_target_dependencies(local_planner ${DEPS})
target_link_libraries(local_planner ${OCTOMAP_LIBRARIES})

# Node 7: Path Follower
add_executable(path_follower src/path_follower_node.cpp)
ament_target_dependencies(path_follower ${DEPS})

# Visualizer (optional)
add_executable(exploration_visualizer src/exploration_visualizer_node.cpp)
ament_target_dependencies(exploration_visualizer ${DEPS})

install(TARGETS
  frontier_detector
  frontier_clusterer
  cost_evaluator
  goal_selector
  global_planner
  local_planner
  path_follower
  exploration_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()