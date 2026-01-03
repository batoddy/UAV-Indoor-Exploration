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
            'rviz_config',
            default_value=rviz_config,
            description='RViz config file path'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
