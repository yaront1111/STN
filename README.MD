# ORION Navigation System
## GPS-Denied Navigation for UAVs

![Status](https://img.shields.io/badge/phase-1%20development-yellow)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)

---

## Project Status

**Current Phase:** Phase 1 - "ORION-Lite"
**Goal:** 5-10 minute GPS-denied flight with <5m drift
**Timeline:** 16 weeks (4 months)
**Strategy:** IMU + LiDAR SLAM with EKF fusion

---

## Quick Links

- [Full ORION Vision](README-ORION.MD) - Complete system specifications
- [Phase 1 Implementation Plan](PHASE1-PLAN.md) - Detailed 16-week roadmap
- [Gemini Technical Guidance](gemini_phase1_guidance.md) - Expert consultation results

---

## What is ORION?

ORION is an AI-powered navigation system that enables UAVs to operate accurately in GPS-denied environments using multi-modal sensor fusion. The full system combines LiDAR, stereo vision, event cameras, magnetometers, and RF sensors with factor graph optimization and Vision Transformer-based terrain matching.

**Phase 1 ("ORION-Lite")** is a simplified but fully functional version focusing on:
- **LiDAR SLAM** (KISS-ICP) for primary odometry
- **IMU** for high-rate motion estimation
- **EKF fusion** (robot_localization) for state estimation
- **PX4 integration** for autonomous flight

---

## System Architecture (Phase 1)

```
Hardware Layer:
┌─────────────────────────────────────────┐
│  Jetson Xavier NX                       │
│  ├─ Livox Mid-360 LiDAR                 │
│  ├─ OAK-D Pro (IMU + Stereo Camera)     │
│  └─ Pixhawk 4/6c Flight Controller      │
└─────────────────────────────────────────┘

Software Stack:
┌─────────────────────────────────────────┐
│  ROS2 Humble                            │
│  ├─ KISS-ICP (LiDAR SLAM)               │
│  ├─ robot_localization (EKF)            │
│  ├─ MAVROS (PX4 Interface)              │
│  └─ PX4 Autopilot (Flight Control)      │
└─────────────────────────────────────────┘
```

---

## Performance Targets (Phase 1)

| Metric | Target | Status |
|--------|--------|--------|
| Position Drift | <5m over 10 min | 🚧 In Development |
| Flight Duration | 5-10 minutes | 🚧 In Development |
| Update Rate | 100Hz | 🚧 In Development |
| Success Rate | >80% | 🚧 In Development |

---

## Project Structure

```
STN/
├── src/                    # ROS2 packages (colcon workspace)
│   ├── orion_bringup/     # Launch files and configurations
│   ├── orion_drivers/     # Sensor drivers
│   ├── orion_slam/        # SLAM integration (KISS-ICP)
│   ├── orion_fusion/      # EKF fusion (robot_localization config)
│   └── orion_control/     # High-level control and planning
├── config/                 # Configuration files
│   ├── sensors/           # Sensor calibrations
│   ├── params/            # ROS2 parameters
│   └── urdf/              # Robot descriptions
├── launch/                 # System launch files
├── scripts/                # Utility scripts
├── tests/                  # Test data and scripts
├── docs/                   # Documentation
└── data/                   # Datasets (DEM, maps, etc.)
    └── dem/srtm/          # SRTM terrain data
```

---

## Hardware Requirements

### Phase 1 Kit (~$1,800)

| Component | Model | Cost |
|-----------|-------|------|
| Compute | Jetson Xavier NX | $399 |
| LiDAR | Livox Mid-360 | $599 |
| Camera+IMU | OAK-D Pro | $349 |
| Flight Controller | Pixhawk 4/6c | $250 |
| Accessories | Power, mounts, cables | $200 |

---

## Software Requirements

- **OS:** Ubuntu 22.04 LTS
- **ROS:** ROS2 Humble
- **Dependencies:**
  - KISS-ICP
  - robot_localization
  - MAVROS
  - PX4 Autopilot
  - Gazebo Fortress (simulation)

---

## Quick Start

### 1. System Setup

```bash
# Clone repository
cd ~/
git clone <repository-url> orion_ws
cd orion_ws

# Install ROS2 Humble
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install ros-humble-robot-localization \
                 ros-humble-mavros \
                 ros-humble-mavros-extras \
                 ros-humble-tf2-tools

# Build workspace
colcon build
source install/setup.bash
```

### 2. Simulation Test

```bash
# Terminal 1: Launch Gazebo sim
ros2 launch orion_bringup simulation.launch.py

# Terminal 2: Launch navigation stack
ros2 launch orion_bringup navigation.launch.py

# Terminal 3: Monitor
rviz2
```

### 3. Hardware Test

```bash
# Launch full system
ros2 launch orion_bringup hardware.launch.py

# Verify sensors
ros2 topic list
ros2 topic echo /imu/data
ros2 topic echo /points2
```

---

## Development Timeline

### Current Week: Week 1-2 (Setup)

- [x] Project structure created
- [x] Documentation framework established
- [ ] Hardware ordered
- [ ] ROS2 workspace setup
- [ ] Sensor drivers installed
- [ ] Initial URDF created

### Next Milestones

- **Week 3-4:** LiDAR SLAM integration
- **Week 5-6:** EKF fusion implementation
- **Week 7-8:** Simulation testing
- **Week 9-12:** Hardware integration and first flights
- **Week 13-16:** Field testing and demo

See [PHASE1-PLAN.md](PHASE1-PLAN.md) for complete timeline.

---

## Contributing

This is an active research and development project. Contributions should align with the Phase 1 plan and maintain code quality standards.

### Development Workflow

1. Create feature branch from `develop`
2. Implement feature with tests
3. Submit PR with clear description
4. Pass CI checks (formatting, build, tests)
5. Code review and merge

---

## Testing

```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select orion_slam

# View test results
colcon test-result --all
```

---

## Documentation

- [Architecture Overview](docs/architecture.md)
- [Hardware Setup Guide](docs/hardware_setup.md)
- [Calibration Procedures](docs/calibration.md)
- [Troubleshooting](docs/troubleshooting.md)

---

## License

Proprietary - All Rights Reserved

---

## Contact

For technical questions or collaboration inquiries, please contact the project team.

---

## Acknowledgments

- **Gemini AI** for technical consultation and implementation guidance
- **KISS-ICP** team for excellent LiDAR SLAM library
- **ROS2** community for robust robotics middleware
- **PX4** project for open-source flight control

---

**Status:** 🚧 Active Development | **Next Update:** Week 2 completion
