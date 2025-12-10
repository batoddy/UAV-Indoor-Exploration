cmake_minimum_required(VERSION 3.8)
project(sim_topic_bridge)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Gazebo depth bridge
add_executable(gz_depth_bridge src/gz_depth_bridge.cpp)
ament_target_dependencies(gz_depth_bridge rclcpp sensor_msgs)
target_link_libraries(gz_depth_bridge gz-transport13::core gz-msgs10::core)

# PX4 to ROS converter
add_executable(px4_to_ros_bridge src/px4_to_ros_bridge.cpp)
ament_target_dependencies(px4_to_ros_bridge rclcpp tf2_ros nav_msgs px4_msgs)

install(TARGETS gz_depth_bridge px4_to_ros_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

# ```
# 
# ---
# 
# ## Dosya Yapısı
# ```
# sim_topic_bridge/
# ├── src/
# |   ├── include/
# |   |   └── px4_to_ros_bridge.hpp
# │   ├── gz_depth_bridge.cpp    
# │   └── px4_to_ros_bridge.cpp         
# ├── launch/
# ├── config/
# ├── CMakeLists.txt
# └── package.xml