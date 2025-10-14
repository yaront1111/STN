"""
Robot Description Launch File for ORION
Publishes URDF and TF static transforms
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
        'config', 'urdf', 'orion_robot.urdf'
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Static TF Publisher (map -> odom)
        # In a real system, this would come from global localization
        # For Phase 1, we assume map and odom are coincident
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])
