"""
Fusion Launch File for ORION
Launches robot_localization EKF and confidence estimator
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    fusion_pkg_dir = get_package_share_directory('orion_fusion')
    ekf_config = os.path.join(fusion_pkg_dir, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # robot_localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('odometry/filtered', '/odom/filtered')
            ]
        ),

        # Confidence Estimator
        Node(
            package='orion_fusion',
            executable='confidence_estimator',
            name='confidence_estimator',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])
