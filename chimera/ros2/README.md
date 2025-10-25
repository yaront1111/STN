# CHIMERA ROS2 Workspace

GPS-denied navigation ROS2 integration for multi-sensor fusion.

## Structure

```
ros2/
├── src/
│   └── chimera_nav/          # Main navigation package
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── default.yaml  # Default parameters
│       ├── launch/
│       │   └── chimera_nav.launch.py
│       ├── src/
│       │   ├── chimera_nav_node.cpp
│       │   └── sensor_bridge.cpp
│       └── include/
└── README.md
```

## Prerequisites

### ROS2 Dependencies
```bash
# ROS2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-desktop \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-tf2-geometry-msgs
```

### CHIMERA Core
First build the CHIMERA core library:
```bash
cd /path/to/chimera
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Build

```bash
# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash

# Build the workspace
cd ros2
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Launch with default parameters
```bash
ros2 launch chimera_nav chimera_nav.launch.py
```

### Launch with custom config
```bash
ros2 launch chimera_nav chimera_nav.launch.py \
  config_file:=my_config.yaml \
  publish_rate:=100.0
```

### Parameters
See `config/default.yaml` for all available parameters:
- Sensor sigmas and thresholds
- Health monitoring configuration
- Mode switching thresholds
- UKF/smoother parameters

## Topics

### Published
- `/chimera/odometry` (nav_msgs/Odometry) - Navigation state @ 50Hz
- `/chimera/diagnostics` (diagnostic_msgs/DiagnosticArray) - System health

### Subscribed
- `/imu/data` (sensor_msgs/Imu) - IMU measurements
- `/lidar/range` (sensor_msgs/Range) - LiDAR altitude
- `/baro/pressure` (sensor_msgs/FluidPressure) - Barometer
- `/mag/field` (sensor_msgs/MagneticField) - Magnetometer

## TF Frames

```
map → odom → base_link
```

- **map**: Fixed world frame
- **odom**: Odometry frame (drift-corrected)
- **base_link**: Robot base frame

## Integration with PX4/MAVROS

The default topic remapping assumes MAVROS is running:

```bash
# Terminal 1: MAVROS
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://:14540@localhost:14557"

# Terminal 2: CHIMERA
ros2 launch chimera_nav chimera_nav.launch.py
```

## nav2 Integration

To use CHIMERA as odometry source for nav2:

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    odom_topic: /chimera/odometry
```

## Monitoring

### View diagnostics
```bash
ros2 topic echo /chimera/diagnostics
```

### Visualize in RViz
```bash
rviz2
# Add → By topic → /chimera/odometry → Odometry
# Add → TF
```

### Check sensor health
```bash
ros2 topic echo /chimera/diagnostics --field status[0].values
```

## Troubleshooting

### Node crashes on startup
- Verify CHIMERA core library is built: `ls ../build/libchimera_core.a`
- Check sensor data is being published: `ros2 topic list`

### No odometry output
- Check IMU buffer filled: `ros2 topic echo /chimera/diagnostics --field status[0].values[1]`
- Node needs 100+ IMU samples before initializing

### Poor altitude accuracy
- Adjust `lidar_sigma_validated` parameter (lower = trust LiDAR more)
- Check for LiDAR stuck detection in diagnostics

## Development

### Run tests
```bash
cd ros2
colcon test --packages-select chimera_nav
colcon test-result --verbose
```

### Lint
```bash
ament_cpplint src/chimera_nav/src/*.cpp
```

## License

Commercial - See main CHIMERA repository LICENSE
