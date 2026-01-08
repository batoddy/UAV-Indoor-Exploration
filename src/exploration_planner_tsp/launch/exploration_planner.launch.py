"""
Exploration Planner - Full Pipeline Launch

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
    rviz_config = os.path.join(pkg_dir, 'config', 'exploration.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'),
        
        # Node 1: Global Tour Planner (TSP)
        Node(
            package='exploration_planner',
            executable='global_tour_planner_node',
            name='global_tour_planner',
            parameters=[{
                'use_sim_time': use_sim_time,
                'v_max': 1.0,
                'yaw_rate_max': 1.0,
                'w_consistency': 0.5,
                'use_2opt': True,
                'replan_threshold': 2.0,
            }],
            output='screen'
        ),
        
        # Node 2: Local Viewpoint Refiner
        Node(
            package='exploration_planner',
            executable='local_refiner_node',
            name='local_refiner',
            parameters=[{
                'use_sim_time': use_sim_time,
                'refine_radius': 5.0,
                'max_refine_clusters': 3,
                'v_max': 1.0,
                'yaw_rate_max': 1.0,
                'w_consistency': 0.5,
            }],
            output='screen'
        ),
        
        # Node 3: Trajectory Generator
        Node(
            package='exploration_planner',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            parameters=[{
                'use_sim_time': use_sim_time,
                'v_max': 1.0,
                'a_max': 0.5,
                'yaw_rate_max': 1.0,
                'yaw_accel_max': 0.5,
                'dt': 0.1,
                'lookahead_time': 5.0,
                'min_waypoint_dist': 0.3,
            }],
            output='screen'
        ),
        
        # Node 4: Trajectory Follower
        Node(
            package='exploration_planner',
            executable='trajectory_follower_node',
            name='trajectory_follower',
            parameters=[{
                'use_sim_time': use_sim_time,
                'control_rate': 20.0,
                'lookahead_dist': 0.5,
                'goal_tolerance_xy': 0.2,
                'goal_tolerance_yaw': 0.1,
                'kp_xy': 1.0,
                'kp_yaw': 1.0,
                'kd_xy': 0.1,
                'kd_yaw': 0.1,
                'v_max': 1.0,
                'yaw_rate_max': 1.0,
                'use_feedforward': True,
                'ff_weight': 0.7,
            }],
            output='screen'
        ),
        
        # Node 5: Visualizer
        Node(
            package='exploration_planner',
            executable='exploration_visualizer_node',
            name='exploration_visualizer',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # RViz (optional)
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