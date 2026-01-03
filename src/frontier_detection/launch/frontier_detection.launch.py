from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('frontier_detection')
    rviz_config = os.path.join(pkg_dir, 'config', 'frontier_detection.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='Occupancy grid topic'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        Node(
            package='frontier_detection',
            executable='frontier_detector_node',
            name='frontier_detector',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_topic': LaunchConfiguration('map_topic'),
                'update_rate': 2.0,
                'min_frontier_size': 5,
                'max_cluster_size': 50,
                'free_threshold': 25,
                'occupied_threshold': 65,
                'pca_split_threshold': 2.0,
            }],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
