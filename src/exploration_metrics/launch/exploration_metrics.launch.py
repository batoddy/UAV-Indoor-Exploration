"""
Launch file for exploration_metrics package.

Usage:
    # Basic launch with map name
    ros2 launch exploration_metrics exploration_metrics.launch.py map:=octomaze

    # This will:
    #   - Look for ground truth at: ground_truth/octomaze/octomaze.yaml
    #   - Save logs to: metric_logs/octomaze/2026-01-24_15-30-22.csv
    #   - Auto-start logging when /exploration/start is called
    #   - Auto-stop logging when /exploration/stop is called

    # Custom comparison mode
    ros2 launch exploration_metrics exploration_metrics.launch.py map:=warehouse comparison_mode:=occupancy_grid
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('exploration_metrics')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='default',
        description='Map name for ground truth lookup and log organization (e.g., octomaze, warehouse)'
    )

    comparison_mode_arg = DeclareLaunchArgument(
        'comparison_mode',
        default_value='occupancy_grid',
        description='Comparison mode: occupancy_grid, octomap, or both'
    )

    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value='/home/batoddy/uav_ws',
        description='Base path for ground truth and log directories'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Metrics publish rate in Hz'
    )

    telemetry_log_rate_arg = DeclareLaunchArgument(
        'telemetry_log_rate',
        default_value='10.0',
        description='Telemetry logging rate in Hz (motion metrics)'
    )

    logging_enabled_arg = DeclareLaunchArgument(
        'logging_enabled',
        default_value='true',
        description='Enable CSV logging (auto-triggered by exploration start/stop)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Exploration metrics node
    exploration_metrics_node = Node(
        package='exploration_metrics',
        executable='exploration_metrics_node',
        name='exploration_metrics_node',
        output='screen',
        parameters=[
            params_file,
            {
                'map_name': LaunchConfiguration('map'),
                'comparison_mode': LaunchConfiguration('comparison_mode'),
                'base_path': LaunchConfiguration('base_path'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'telemetry_log_rate': LaunchConfiguration('telemetry_log_rate'),
                'logging_enabled': LaunchConfiguration('logging_enabled'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    # Map saver node
    map_saver_node = Node(
        package='exploration_metrics',
        executable='map_saver_node',
        name='map_saver_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    return LaunchDescription([
        # Launch arguments
        map_arg,
        comparison_mode_arg,
        base_path_arg,
        publish_rate_arg,
        telemetry_log_rate_arg,
        logging_enabled_arg,
        use_sim_time_arg,

        # Nodes
        exploration_metrics_node,
        map_saver_node,
    ])
