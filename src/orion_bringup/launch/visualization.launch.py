"""
Visualization Launch File for ORION
Launches RViz2 with ORION configuration
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
    rviz_config = os.path.join(pkg_dir, 'rviz', 'orion.rviz')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        ),
    ])
