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

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                
                # ============ TEMEL AYARLAR ============
                'resolution': 0.20,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'transform_tolerance' : 1.0,
                # ============ SENSÖR MODELİ ============
                'sensor_model.max_range': 7.5,         # Azaltıldı - uzak noktalar güvenilmez
                'sensor_model.min_range': 0.1,
                'sensor_model.hit': 0.55,              # 0.6'dan düşür - tek ölçümde daha az güven
                'sensor_model.miss': 0.40,             # 0.45'den düşür - boş alan daha agresif temizlesin
                'sensor_model.min': 0.12,              
                'sensor_model.max': 0.90,              

                # ============ GÜRÜLTÜ FİLTRELEME ============
                'filter_speckles': True,
                'filter_speckles_size': 5,             # İzole vokselleri temizle
                
                # ============ Z EKSENİ FİLTRELEME ============
                'pointcloud_min_z': -0.5,
                'pointcloud_max_z': 10.0,
                    
                # 2D HARİTAYA PROJEKSİYON
                'occupancy_min_z': 1.0,               # 1m altı engel sayılmaz
                'occupancy_max_z': 1.8,              # 1.8m üstü engel sayılmaz
                
                # ============ ZEMİN FİLTRELEME ============
                'filter_ground': True,
                'ground_filter.distance': 0.5,        # Zemin algılama mesafesi
                'ground_filter.angle': 0.35,          # Zemin açısı toleransı (~14°)
                'ground_filter.plane_distance': 0.25, # Düzlem toleransı
                    
                # ============ YAYINLAMA AYARLARI ============
                'latch': True,
                'publish_free_space': True,
                'publish_unknown_space': True,
                'height_map': True,
                'colored_map': False,

                'automatic_expansion': True,          
                'expand_rate': 10.0,           
            }],
            remappings=[
                ('cloud_in', '/camera/points'),
                ('projected_map', '/projected_map'),
                ('octomap_binary', '/octomap_binary'),
                ('octomap_full', '/octomap_full'),
                ('octomap_point_cloud_centers', '/octomap_points'),
            ],
            output='screen'
        ),

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
