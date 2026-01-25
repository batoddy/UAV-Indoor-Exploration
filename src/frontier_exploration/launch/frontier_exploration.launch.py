"""
Frontier Exploration - Full Pipeline Launch

Pipeline:
  /projected_map -> [Frontier Detector] -> /frontier_clusters
                 -> [Viewpoint Generator] -> /frontier_clusters_with_viewpoints
                 -> [Visualizer] -> /frontier_markers, /fis_info

Usage:
  ros2 launch frontier_exploration frontier_exploration.launch.py
  ros2 launch frontier_exploration frontier_exploration.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('frontier_exploration')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Node 1: Frontier Detector
        Node(
            package='frontier_exploration',
            executable='frontier_detector_node',
            name='frontier_detector',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Node 2: Viewpoint Generator
        Node(
            package='frontier_exploration',
            executable='viewpoint_generator_node',
            name='viewpoint_generator',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Node 3: Visualizer
        Node(
            package='frontier_exploration',
            executable='frontier_visualizer_node',
            name='frontier_visualizer',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
