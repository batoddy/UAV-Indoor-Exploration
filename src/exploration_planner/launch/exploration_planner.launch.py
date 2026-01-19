"""
Exploration Planner - Full Pipeline Launch with Nav2

Usage:
  ros2 launch exploration_planner exploration_planner.launch.py
  ros2 launch exploration_planner exploration_planner.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('exploration_planner')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'exploration.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'),

        # ============================================================
        # NAV2 PLANNER SERVER (includes global costmap)
        # ============================================================
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['planner_server']
            }],
            output='screen'
        ),

        # ============================================================
        # EXPLORATION NODES
        # ============================================================

        # Greedy Frontier Selector
        Node(
            package='exploration_planner',
            executable='global_tour_planner_node',
            name='greedy_frontier_selector',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Nav2 Path Planner Node (our wrapper)
        Node(
            package='exploration_planner',
            executable='nav2_path_planner_node',
            name='nav2_path_planner',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Path Yaw Injector
        Node(
            package='exploration_planner',
            executable='path_yaw_injector_node',
            name='path_yaw_injector',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Trajectory Generator
        Node(
            package='exploration_planner',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Trajectory Follower
        Node(
            package='exploration_planner',
            executable='trajectory_follower_node',
            name='trajectory_follower',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Visualizer
        Node(
            package='exploration_planner',
            executable='exploration_visualizer_node',
            name='exploration_visualizer',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ============================================================
        # OPTIONAL
        # ============================================================

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen'
        ),
    ])
