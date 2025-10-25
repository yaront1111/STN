"""CHIMERA SITL Launch File

Launches CHIMERA navigation with PX4 SITL integration via MAVROS.

This launch file sets up:
- MAVROS node (connects to PX4 SITL on UDP 14540)
- CHIMERA navigation node (processes sensor data from MAVROS)
- Optional RViz visualization

Prerequisites:
1. Start PX4 SITL first:
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic_iris

2. Then launch this file:
   ros2 launch chimera_nav chimera_sitl.launch.py

Topic Mapping:
- IMU: /mavros/imu/data -> CHIMERA
- Barometer: /mavros/imu/static_pressure -> CHIMERA
- Magnetometer: /mavros/imu/mag -> CHIMERA
- LiDAR: /tfmini/range -> CHIMERA (simulated or real)

Output:
- Odometry: /chimera/odometry
- Diagnostics: /chimera/diagnostics
- TF: odom -> base_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ========== Launch Arguments ==========

    # MAVROS configuration
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@localhost:14557',
        description='MAVROS FCU URL (default: PX4 SITL UDP port)'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='MAVROS GCS URL (optional)'
    )

    # CHIMERA configuration
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='CHIMERA odometry publish rate (Hz)'
    )

    # Visualization
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # Frame IDs
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

    # ========== MAVROS Node ==========

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                # Plugin configuration
                'plugin_allowlist': [
                    'sys_status',
                    'sys_time',
                    'imu',
                    'local_position',
                    'global_position',
                    'setpoint_position',
                    'setpoint_velocity',
                    'setpoint_attitude',
                    'command',
                ],
                # IMU configuration
                'imu/frame_id': 'imu_link',
                'imu/angular_velocity_stdev': 0.0003490659,  # 0.02 deg/s
                'imu/linear_acceleration_stdev': 0.0003,
                'imu/orientation_stdev': 0.0,
                'imu/magnetic_stdev': 0.0,
            }
        ],
        remappings=[
            # Remap MAVROS topics to match CHIMERA expectations
            ('/mavros/imu/data', '/mavros/imu/data'),
            ('/mavros/imu/static_pressure', '/baro/pressure'),
            ('/mavros/imu/mag', '/mag/field'),
        ]
    )

    # ========== CHIMERA Navigation Node ==========

    chimera_node = Node(
        package='chimera_nav',
        executable='chimera_nav_node',
        name='chimera_nav',
        output='screen',
        parameters=[
            {
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id_map': LaunchConfiguration('frame_map'),
                'frame_id_odom': LaunchConfiguration('frame_odom'),
                'frame_id_base': LaunchConfiguration('frame_base'),
                # Sensor parameters (tuned for PX4 SITL)
                'lidar_sigma_validated': 0.05,
                'lidar_sigma_coldstart': 0.2,
                'baro_sigma': 1.5,
                'mag_sigma': 0.1,
                'of_sigma_px': 2.0,
                # Health thresholds
                'lidar_stuck_eps': 0.01,
                'lidar_stuck_spread': 0.005,
                'mag_gate_frac': 0.30,
                # Mode switching
                'baro_takeover_thresh_m': 50.0,
            }
        ],
        remappings=[
            ('imu/data', '/mavros/imu/data'),
            ('lidar/range', '/tfmini/range'),
            ('baro/pressure', '/baro/pressure'),
            ('mag/field', '/mag/field'),
        ]
    )

    # ========== Simulated LiDAR (TFmini) ==========
    # NOTE: Replace this with real sensor or Gazebo plugin in production

    simulated_lidar_node = Node(
        package='chimera_nav',
        executable='simulated_lidar.py',
        name='simulated_lidar',
        output='screen',
        parameters=[
            {
                'publish_rate': 20.0,  # 20 Hz
                'frame_id': 'lidar_link',
                'min_range': 0.3,
                'max_range': 12.0,
                'field_of_view': 0.04,  # 2.3 degrees (TFmini spec)
            }
        ],
        # Only launch if you don't have a real LiDAR sensor
        # Comment out this node if using Gazebo LiDAR plugin
        condition=IfCondition('false')  # Disabled by default
    )

    # ========== RViz Visualization ==========

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('chimera_nav'),
        'rviz',
        'chimera_sitl.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ========== Static Transforms ==========
    # Define sensor positions relative to base_link

    # base_link -> imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # base_link -> lidar_link (mounted downward)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['0', '0', '-0.05', '0', '0', '0', 'base_link', 'lidar_link']
    )

    # ========== Launch Description ==========

    return LaunchDescription([
        # Arguments
        fcu_url_arg,
        gcs_url_arg,
        publish_rate_arg,
        use_rviz_arg,
        frame_map_arg,
        frame_odom_arg,
        frame_base_arg,

        # Nodes
        mavros_node,
        chimera_node,
        # simulated_lidar_node,  # Uncomment if needed
        static_tf_imu,
        static_tf_lidar,
        rviz_node,
    ])
