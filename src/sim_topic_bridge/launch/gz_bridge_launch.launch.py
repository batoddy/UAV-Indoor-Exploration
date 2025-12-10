from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_topic_bridge',
            executable='gz_depth_bridge',
            name='gz_depth_bridge',
            output='screen'
        ),
    ])