"""
Frontier Exploration - Full Pipeline Launch

Pipeline:
  /map -> [Frontier Detector] -> /frontier_clusters
       -> [Viewpoint Generator] -> /frontier_clusters_with_viewpoints  
       -> [Cost Computer] -> /frontier_clusters_complete
       -> [Visualizer] -> /frontier_markers, /fis_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_topic = LaunchConfiguration('map_topic', default='/map')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        
        # Node 1: Frontier Detector
        Node(
            package='frontier_exploration',
            executable='frontier_detector_node',
            name='frontier_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_topic': map_topic,
                'min_frontier_size': 5,
                'max_cluster_size': 50,
                'free_threshold': 25,
                'occupied_threshold': 65,
                'pca_split_threshold': 2.0,
            }],
            output='screen'
        ),
        
        # Node 2: Viewpoint Generator
        Node(
            package='frontier_exploration',
            executable='viewpoint_generator_node',
            name='viewpoint_generator',
            parameters=[{
                'use_sim_time': use_sim_time,
                'sensor_range': 5.0,
                'sensor_fov_h': 1.57,
                'min_dist': 1.0,
                'max_dist': 4.0,
                'num_dist_samples': 3,
                'num_angle_samples': 12,
                'min_coverage': 3,
                'max_viewpoints': 5,
                'free_threshold': 25,
                'occupied_threshold': 65,
            }],
            output='screen'
        ),
        
        # Node 3: Cost Computer
        Node(
            package='frontier_exploration',
            executable='cost_computer_node',
            name='cost_computer',
            parameters=[{
                'use_sim_time': use_sim_time,
                'v_max': 1.0,
                'yaw_rate_max': 1.0,
            }],
            output='screen'
        ),
        
        # Node 4: Visualizer
        Node(
            package='frontier_exploration',
            executable='frontier_visualizer_node',
            name='frontier_visualizer',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
    ])