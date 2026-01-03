from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('frontier_exploration')
    config = os.path.join(pkg_dir, 'config', 'exploration_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Node 1: Frontier Detector
        Node(
            package='frontier_exploration',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Node 2: Frontier Clusterer
        Node(
            package='frontier_exploration',
            executable='frontier_clusterer',
            name='frontier_clusterer',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Node 3: Cost Evaluator
        Node(
            package='frontier_exploration',
            executable='cost_evaluator',
            name='cost_evaluator',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Node 4: Goal Selector
        Node(
            package='frontier_exploration',
            executable='goal_selector',
            name='goal_selector',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Node 5: Global Planner
        Node(
            package='frontier_exploration',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Node 6: Local Planner
        Node(
            package='frontier_exploration',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Node 7: Path Follower
        Node(
            package='frontier_exploration',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Visualizer
        Node(
            package='frontier_exploration',
            executable='exploration_visualizer',
            name='exploration_visualizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
