"""
Exploration Planner - Full Pipeline Launch with Nav2

Usage:
  ros2 launch exploration_planner exploration_planner.launch.py
  ros2 launch exploration_planner exploration_planner.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('exploration_planner')
    frontier_pkg_dir = get_package_share_directory('frontier_exploration')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    frontier_params_file = os.path.join(frontier_pkg_dir, 'config', 'params.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'exploration.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'),

        # ============================================================
        # EXPLORATION PLANNER NODES
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
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Telemetry
        Node(
            package='exploration_planner',
            executable='telemetry_node',
            name='telemetry_node',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

             # ============================================================
        # NAV2 PLANNER SERVER ONLY (controller, bt_navigator not used)
        # ============================================================
        # Planner Server (SmacPlanner2D for global path planning)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Lifecycle Manager (only for planner_server)
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
        # RVIZ (optional)
        # ============================================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen',
            # env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])
