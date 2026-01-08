"""
Exploration Planner - Full Pipeline Launch

Tüm parametreler config/params.yaml dosyasından çekilir.

Pipeline:
  1. global_tour_planner → viewpoint seçimi
  2. path_planner → global path (A*/RRT)
  3. local_path_adjuster → pointcloud ile local obstacle avoidance
  4. trajectory_generator → smooth trajectory
  5. trajectory_follower → cmd_vel

Usage:
  ros2 launch exploration_planner exploration_planner.launch.py
  ros2 launch exploration_planner exploration_planner.launch.py rviz:=true
  ros2 launch exploration_planner exploration_planner.launch.py planner:=rrt

Local path adjuster kontrolü params.yaml'dan yapılır:
  local_path_adjuster_enabled: true/false
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
    rviz_config = os.path.join(pkg_dir, 'config', 'exploration.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='false')
    planner = LaunchConfiguration('planner', default='astar')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'),
        DeclareLaunchArgument('planner', default_value='astar', description='Path planner: astar or rrt'),
        
        # Node 0: Costmap Generator (OctoMap → 2D)
        Node(
            package='exploration_planner',
            executable='costmap_generator_node',
            name='costmap_generator',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Node 1: Greedy Frontier Selector
        Node(
            package='exploration_planner',
            executable='global_tour_planner_node',
            name='greedy_frontier_selector',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Node 2: Path Planner (A* or RRT)
        Node(
            package='exploration_planner',
            executable='path_planner_node',
            name='path_planner',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'planner_type': planner}],
            output='screen'
        ),
        
        # Node 3: Local Path Adjuster (Pointcloud obstacle avoidance)
        # Enable/disable via params.yaml: local_path_adjuster_enabled: true/false
        Node(
            package='exploration_planner',
            executable='local_path_adjuster_node',
            name='local_path_adjuster',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Node 4: Local Viewpoint Refiner
        Node(
            package='exploration_planner',
            executable='local_refiner_node',
            name='local_refiner',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Node 5: Trajectory Generator
        # Listens to both /planned_path and /adjusted_path
        # Uses adjusted_path if local_path_adjuster is running, otherwise planned_path
        Node(
            package='exploration_planner',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Node 6: Trajectory Follower
        Node(
            package='exploration_planner',
            executable='trajectory_follower_node',
            name='trajectory_follower',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Node 7: Visualizer
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
