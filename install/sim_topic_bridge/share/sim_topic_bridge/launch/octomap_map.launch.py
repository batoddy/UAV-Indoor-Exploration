from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # OctoMap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            parameters=[{
                'resolution': 0.05,
                'frame_id': 'map',
                'sensor_model.max_range': 5.0,
            }],
            remappings=[('cloud_in', '/camera/points')]
        ),
    ])