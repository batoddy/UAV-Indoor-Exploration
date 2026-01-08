import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('uav_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    flight_altitude = LaunchConfiguration('flight_altitude', default='2.0')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Nav2 params file
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'flight_altitude',
            default_value='5.0',
            description='Fixed flight altitude in meters'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description='Automatically start Nav2 lifecycle nodes'
        ),
        
             
 # Octomap to 2D map projection for Nav2
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
        
        # ============ SENSÖR MODELİ ============
        'sensor_model.max_range': 5.0,         # Azaltıldı - uzak noktalar güvenilmez
        'sensor_model.min_range': 0.3,
        'sensor_model.hit': 0.55,              # 0.6'dan düşür - tek ölçümde daha az güven
        'sensor_model.miss': 0.40,             # 0.45'den düşür - boş alan daha agresif temizlesin
        'sensor_model.min': 0.12,              # Aynı kalabilir
        'sensor_model.max': 0.90,              # 0.97'den düşür - kesinlik üst limiti

        # ============ GÜRÜLTÜ FİLTRELEME ============
        'filter_speckles': True,
        'filter_speckles_size': 5,             # Ekle - izole vokselleri temizle
        
        # ============ Z EKSENİ FİLTRELEME ============
        'pointcloud_min_z': -1.0,
        'pointcloud_max_z': 10.0,
        
        # ⭐ 2D HARİTAYA PROJEKSİYON
        'occupancy_min_z': 0.8,               # ⭐ 50cm altı engel sayılmaz
        'occupancy_max_z': 1.9,              # ⭐ 1.5m üstü engel sayılmaz
        
        # ============ ZEMİN FİLTRELEME ============
        'filter_ground': True,
        'ground_filter.distance': 0.5,        # ⭐ Zemin algılama mesafesi
        'ground_filter.angle': 0.35,          # ⭐ Zemin açısı toleransı (~14°)
        'ground_filter.plane_distance': 0.25, # ⭐ Düzlem toleransı
        
        # ============ GÜRÜLTÜ FİLTRELEME ============
        'filter_speckles': True,
        
        # ============ YAYINLAMA AYARLARI ============
        'latch': True,
        'publish_free_space': True,
        'height_map': True,
        'colored_map': False,
        'incremental_2D_projection': False,
    }],
    remappings=[
        ('cloud_in', '/camera/points'),
        ('projected_map', '/map'),
        ('octomap_binary', '/octomap_binary'),
        ('octomap_full', '/octomap_full'),
        ('octomap_point_cloud_centers', '/octomap_points'),
    ],
    output='screen'
),
        
        # # Nav2 navigation stack
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        #     ),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'params_file': nav2_params_file,
        #         'autostart': autostart,
        #     }.items()
        # ),
        
        
        # # RViz (optional - comment out if running separately)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2_drone.rviz')],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),

    ])