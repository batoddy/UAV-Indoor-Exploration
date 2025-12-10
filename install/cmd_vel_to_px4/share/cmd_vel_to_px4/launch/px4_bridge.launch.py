from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('takeoff_altitude', default_value='4.0'),

        # Odom Converter (PX4 NED -> ROS ENU + TF)
        Node(
            package='cmd_vel_to_px4',
            executable='odom_converter.py',
            name='odom_converter',
            output='screen',
        ),

        Node(
            package='sim_topic_bridge',
            executable='gz_depth_bridge',
            name='gz_depth_bridge',
            output='screen'
        ),

        # Offboard Control (arm, takeoff, cmd_vel)
        Node(
            package='cmd_vel_to_px4',
            executable='cmd_vel_px4_bridge',
            name='cmd_vel_px4_bridge',
            # output='screen',
            parameters=[{
                'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0',
                       '0', '0', '0',
                       'camera_link', 'camera_optical_frame']
        )
        
    ])