"""
Hardware Launch File for ORION
Launches all sensor drivers
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('orion_bringup')
    config_dir = os.path.join(pkg_dir, 'config')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # IMU Driver
        Node(
            package='orion_drivers',
            executable='imu_driver',
            name='imu_driver',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': '/imu/data',
                'health_topic': '/imu/health',
                'frame_id': 'imu_link',
                'publish_rate': 400.0
            }]
        ),

        # LiDAR Driver
        Node(
            package='orion_drivers',
            executable='lidar_driver',
            name='lidar_driver',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'lidar_topic': '/lidar/points',
                'health_topic': '/lidar/health',
                'frame_id': 'lidar_link',
                'publish_rate': 10.0
            }]
        ),

        # Health Monitor
        Node(
            package='orion_drivers',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'sensors': ['imu', 'lidar'],
                'diagnostics_topic': '/diagnostics',
                'publish_rate': 1.0
            }]
        ),
    ])
