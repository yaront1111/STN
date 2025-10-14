"""
SLAM Launch File for ORION
Launches KISS-ICP LiDAR SLAM
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    slam_pkg_dir = get_package_share_directory('orion_slam')
    slam_config = os.path.join(slam_pkg_dir, 'config', 'kiss_icp_params.yaml')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # KISS-ICP Wrapper
        Node(
            package='orion_slam',
            executable='kiss_icp_wrapper',
            name='kiss_icp_wrapper',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
    ])
