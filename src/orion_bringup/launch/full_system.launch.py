"""
Full System Launch File for ORION
Launches all components: hardware, SLAM, fusion, and visualization
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('orion_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Include hardware launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'hardware.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Include SLAM launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'slam.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Include fusion launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'fusion.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Include visualization launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'visualization.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
