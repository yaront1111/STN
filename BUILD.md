# Build Instructions

## Prerequisites

**ROS2 Humble on Ubuntu 22.04** (native Linux or WSL2)

## Quick Build

```bash
cd ~/orion_ws  # or wherever you cloned to

# Install dependencies
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    python3-numpy

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## Test

```bash
# Test hardware drivers (dummy data)
ros2 launch orion_bringup hardware.launch.py

# In another terminal, verify topics
ros2 topic list
ros2 topic echo /imu/data --once
ros2 topic echo /lidar/points --once
```

## Validated

✅ All Python syntax checked
✅ Package structure correct
✅ Ready for ROS2 build

## Note

Current development on Windows. Need Linux/WSL2 for actual ROS2 build.
