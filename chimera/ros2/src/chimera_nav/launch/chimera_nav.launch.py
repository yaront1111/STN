"""CHIMERA Navigation ROS2 Launch File

Launches the CHIMERA navigation node with configurable parameters.
Supports single-sensor and multi-sensor configurations.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default.yaml',
        description='CHIMERA configuration file'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Odometry publish rate (Hz)'
    )

    frame_map_arg = DeclareLaunchArgument(
        'frame_map',
        default_value='map',
        description='Map frame ID'
    )

    frame_odom_arg = DeclareLaunchArgument(
        'frame_odom',
        default_value='odom',
        description='Odometry frame ID'
    )

    frame_base_arg = DeclareLaunchArgument(
        'frame_base',
        default_value='base_link',
        description='Base link frame ID'
    )

    # Get package share directory
    pkg_share = FindPackageShare('chimera_nav')

    # CHIMERA navigation node
    chimera_node = Node(
        package='chimera_nav',
        executable='chimera_nav_node',
        name='chimera_nav',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('config_file')]),
            {
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id_map': LaunchConfiguration('frame_map'),
                'frame_id_odom': LaunchConfiguration('frame_odom'),
                'frame_id_base': LaunchConfiguration('frame_base'),
            }
        ],
        remappings=[
            ('imu/data', '/mavros/imu/data'),
            ('lidar/range', '/tfmini/range'),
            ('baro/pressure', '/mavros/imu/static_pressure'),
            ('mag/field', '/mavros/imu/mag'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        publish_rate_arg,
        frame_map_arg,
        frame_odom_arg,
        frame_base_arg,
        chimera_node,
    ])
