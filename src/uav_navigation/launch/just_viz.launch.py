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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    flight_altitude = LaunchConfiguration('flight_altitude', default='2.0')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Nav2 params file
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
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
        
        # Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.1,
                'frame_id': 'map',
                'sensor_model.max_range': 5.0,
                'sensor_model.hit': 0.7,
                'sensor_model.miss': 0.4,
                'sensor_model.min': 0.12,
                'sensor_model.max': 0.97,
                'latch': False,
                'filter_ground': False,
                'filter_speckles': True,
                'ground_filter.distance': 0.04,
                'ground_filter.angle': 0.15,
                'ground_filter.plane_distance': 0.07,
                'pointcloud_min_z': -1.0,
                'pointcloud_max_z': 10.0,
                'occupancy_min_z': 0.1,
                'occupancy_max_z': 5.0,
            }],
            remappings=[
                ('cloud_in', '/camera/points')
            ],
            output='screen'
        ),
        
        # Octomap to 2D map projection for Nav2
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_to_gridmap',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.1,
                'frame_id': 'map',
                'sensor_model.max_range': 5.0,
            }],
            remappings=[
                ('cloud_in', '/camera/points'),
                ('projected_map', '/map')
            ],
            output='screen'
        ),
        
        # Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': autostart,
            }.items()
        ),
        
        # Nav2 to PX4 bridge
        Node(
            package='uav_navigation',
            executable='nav2_to_px4_bridge',
            name='nav2_to_px4_bridge',
            parameters=[{
                'use_sim_time': use_sim_time,
                'flight_altitude': flight_altitude,
                'velocity_scale': 1.0,
                'cmd_vel_timeout': 0.5,
                'auto_arm': True,
            }],
            output='screen'
        ),
        
        # RViz (optional - comment out if running separately)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2_drone.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        # Launch dosyasında octomap_to_gridmap node'unu şu şekilde güncelle:
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_to_gridmap',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.1,
                'frame_id': 'map',
                'sensor_model.max_range': 5.0,
                'latch': True,  # Bu önemli - map_server gibi davranması için
            }],
            remappings=[
                ('cloud_in', '/camera/points'),
                ('projected_map', '/map')
            ],
            output='screen'
        ),
    ])