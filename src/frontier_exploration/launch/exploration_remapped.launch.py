from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file with topic remapping for custom setups.
    
    Adjust remappings based on your system:
    - octomap_topic: Your OctoMap topic name
    - odom_topic: Your odometry topic
    - cmd_vel_topic: Your velocity command topic
    """
    pkg_dir = get_package_share_directory('frontier_exploration')
    config = os.path.join(pkg_dir, 'config', 'exploration_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'exploration.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('rviz')
    
    # Common remappings - adjust these for your setup
    octomap_remap = ('/octomap_binary', '/octomap_binary')
    odom_remap = ('/odom', '/odom')  # Change to your odom topic
    cmd_vel_remap = ('/cmd_vel', '/cmd_vel')  # Change to your cmd_vel topic
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),

        # Frontier Detector
        Node(
            package='frontier_exploration',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[octomap_remap],
        ),
        
        # Frontier Clusterer
        Node(
            package='frontier_exploration',
            executable='frontier_clusterer',
            name='frontier_clusterer',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        
        # Cost Evaluator
        Node(
            package='frontier_exploration',
            executable='cost_evaluator',
            name='cost_evaluator',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[odom_remap],
        ),
        
        # Goal Selector
        Node(
            package='frontier_exploration',
            executable='goal_selector',
            name='goal_selector',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[odom_remap],
        ),
        
        # Global Planner
        Node(
            package='frontier_exploration',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[octomap_remap, odom_remap],
        ),
        
        # Local Planner
        Node(
            package='frontier_exploration',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[octomap_remap, odom_remap],
        ),
        
        # Path Follower
        Node(
            package='frontier_exploration',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[odom_remap, cmd_vel_remap],
        ),
        
        # Visualizer
        Node(
            package='frontier_exploration',
            executable='exploration_visualizer',
            name='exploration_visualizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # RViz
        Node(
            condition=IfCondition(launch_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

    ])
