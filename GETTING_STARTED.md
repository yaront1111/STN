# Getting Started with ORION-Lite Development

## Welcome!

You're about to start building a GPS-denied navigation system for UAVs. This guide will help you get started with Phase 1 development.

---

## 📋 Prerequisites

Before you begin, ensure you have:

- [ ] **Hardware ordered** (or access to borrowed/existing hardware)
- [ ] **Development machine** with Ubuntu 22.04 (native or VM with at least 8GB RAM, 50GB disk)
- [ ] **Basic knowledge** of ROS2, C++, and Python
- [ ] **Git** configured for version control
- [ ] **Time commitment** of 10-20 hours/week for 16 weeks

---

## 🎯 What You're Building

**Phase 1 Goal:** A UAV that can fly autonomously for 5-10 minutes without GPS, maintaining <5m position accuracy.

**Key Components:**
- LiDAR SLAM for position tracking
- IMU for motion estimation
- EKF fusion for state estimation
- PX4 autopilot for flight control

---

## 📚 Essential Reading

Before writing code, read these documents:

1. **[README-ORION.MD](README-ORION.MD)** - Full system vision (15 min)
2. **[PHASE1-PLAN.md](PHASE1-PLAN.md)** - Detailed implementation plan (30 min)
3. **[gemini_phase1_guidance.md](gemini_phase1_guidance.md)** - Expert technical guidance (20 min)

**Total reading time: ~1 hour**

---

## 🛠️ Development Environment Setup

### Step 1: Install Ubuntu 22.04

If you don't have Ubuntu 22.04:
- **Option A:** Dual boot (best performance)
- **Option B:** VMware/VirtualBox VM (8GB RAM minimum)
- **Option C:** WSL2 on Windows (limited hardware support)

### Step 2: Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    sudo apt-key add -

sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
    /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 3: Setup Workspace

```bash
# Create workspace
mkdir -p ~/orion_ws/src
cd ~/orion_ws

# Clone the repository (adjust URL)
cd src
git clone <your-repo-url> orion

# Install dependencies
cd ~/orion_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Add to bashrc
echo "source ~/orion_ws/install/setup.bash" >> ~/.bashrc
```

### Step 4: Install Phase 1 Dependencies

```bash
# Core navigation stack
sudo apt install ros-humble-robot-localization \
                 ros-humble-tf2-tools \
                 ros-humble-rqt \
                 ros-humble-plotjuggler-ros

# Visualization
sudo apt install ros-humble-rviz2 \
                 ros-humble-rqt-*

# MAVROS (PX4 communication)
sudo apt install ros-humble-mavros \
                 ros-humble-mavros-extras

# Download GeographicLib datasets (required for MAVROS)
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# KISS-ICP (will install from source later in Week 3)
# Instructions: https://github.com/PRBonn/kiss-icp
```

### Step 5: Simulation Setup

```bash
# Install Gazebo Fortress
sudo apt-get update
sudo apt-get install ros-humble-ros-gz

# Install PX4 Autopilot
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500
```

### Step 6: Development Tools

```bash
# VS Code (recommended IDE)
sudo snap install code --classic

# Install VS Code extensions (via Extensions panel):
# - ROS
# - C/C++
# - Python
# - CMake
# - GitLens

# Alternative: CLion (for C++ development)
# sudo snap install clion --classic
```

---

## 🔍 Verify Installation

Run these commands to verify everything is installed:

```bash
# ROS2
ros2 --version  # Should show: ros2 cli version: 0.18.x

# Check environment
printenv | grep ROS  # Should show ROS_DISTRO=humble

# Colcon
colcon --version  # Should show colcon version

# MAVROS
ros2 pkg list | grep mavros  # Should show mavros packages

# Gazebo
gz sim --version  # Should show Gazebo version
```

---

## 📦 Hardware Preparation

### Week 1-2 Hardware Checklist

Order these components (see PHASE1-PLAN.md for details):

- [ ] **Jetson Xavier NX Dev Kit** ($399)
  - https://developer.nvidia.com/embedded/jetson-xavier-nx-devkit
  - Include 64GB+ SD card and power supply

- [ ] **Livox Mid-360 LiDAR** ($599)
  - https://www.livoxtech.com/mid-360
  - Includes cables and mounting bracket

- [ ] **OAK-D Pro Camera** ($349)
  - https://shop.luxonis.com/products/oak-d-pro
  - Includes USB-C cable

- [ ] **Pixhawk 4 or 6c** ($250)
  - https://holybro.com/products/pixhawk-4
  - Include GPS module, telemetry radio, power module

- [ ] **UAV Frame** (if not using existing)
  - 450mm-550mm class quadcopter frame
  - Motors, ESCs, battery suitable for 5-25kg payload

- [ ] **Power Distribution**
  - 5V buck converter for Jetson (5A+)
  - 12V output for LiDAR
  - 5V for camera

- [ ] **Mounting Hardware**
  - Vibration dampers
  - Mounting plates/brackets
  - Cable management (zip ties, velcro)

**Budget Total: ~$1,800-$2,000**

---

## 🎓 Learning Resources

### Must-Watch Videos

1. **ROS2 Basics:**
   - ROS2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
   - ROS2 Navigation2: https://navigation.ros.org/

2. **SLAM Fundamentals:**
   - Cyrill Stachniss SLAM Course: https://www.youtube.com/watch?v=U6vr3iNrwRA
   - Understanding LiDAR SLAM: https://www.youtube.com/watch?v=eYN0FBDqAlg

3. **Kalman Filtering:**
   - Kalman Filter Explained: https://www.kalmanfilter.net/
   - robot_localization Tutorial: http://docs.ros.org/en/noetic/api/robot_localization/html/

4. **PX4 Development:**
   - PX4 Dev Guide: https://dev.px4.io/
   - MAVROS Offboard Control: https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html

### Recommended Books

- **"Probabilistic Robotics"** by Thrun, Burgard, Fox (the bible)
- **"Multiple View Geometry in Computer Vision"** by Hartley & Zisserman
- **"State Estimation for Robotics"** by Timothy Barfoot

---

## 📝 Week 1-2 Tasks

Now that your environment is set up, here's what to focus on:

### Week 1: Environment & Simulation

- [ ] Complete all installation steps above
- [ ] Verify PX4 SITL launches successfully
- [ ] Fly simulated drone in Gazebo (manual control)
- [ ] Familiarize yourself with RViz2 and PlotJuggler
- [ ] Read all Phase 1 documentation

### Week 2: Hardware & Workspace

- [ ] Receive and unbox hardware
- [ ] Flash Jetson Xavier NX with Ubuntu 22.04
- [ ] Install ROS2 on Jetson
- [ ] Bench test each sensor individually:
  - [ ] LiDAR: See point cloud in RViz
  - [ ] Camera: View image streams
  - [ ] IMU: Read orientation/acceleration
  - [ ] Pixhawk: Connect via MAVROS
- [ ] Create initial URDF for your UAV
- [ ] Set up TF tree with sensor transforms

**Deliverable:** All sensors publishing data, visible in RViz

---

## 🚀 Next Steps

Once Week 1-2 tasks are complete, proceed to:

1. **Week 3-4:** KISS-ICP LiDAR SLAM integration
2. **Week 5-6:** EKF fusion implementation
3. **Week 7-8:** PX4 integration and simulation testing

See [PHASE1-PLAN.md](PHASE1-PLAN.md) for detailed week-by-week instructions.

---

## ❓ Troubleshooting

### Common Issues

**ROS2 commands not found:**
```bash
source /opt/ros/humble/setup.bash
source ~/orion_ws/install/setup.bash
```

**colcon build fails:**
```bash
# Clean workspace
rm -rf build install log
colcon build --symlink-install
```

**Jetson won't flash:**
- Use official JetPack SDK Manager: https://developer.nvidia.com/embedded/jetpack

**PX4 SITL won't launch:**
- Ensure prerequisites installed: `bash ./Tools/setup/ubuntu.sh --no-nuttx`
- Try: `make clean` then rebuild

---

## 📞 Getting Help

If you're stuck:

1. **Check documentation** in `docs/` folder
2. **Review Gemini guidance** in `gemini_phase1_guidance.md`
3. **Search issues** in project repository
4. **ROS Answers**: https://answers.ros.org/
5. **PX4 Forums**: https://discuss.px4.io/

---

## ✅ Checklist: Ready to Start?

Before proceeding to Week 3, ensure:

- [ ] ROS2 Humble fully installed and sourced
- [ ] Workspace building successfully
- [ ] All hardware received and bench tested
- [ ] Each sensor publishing data in RViz
- [ ] PX4 SITL flying in Gazebo
- [ ] URDF created with correct sensor transforms
- [ ] All Phase 1 documentation read and understood

**Once all boxes are checked, you're ready to start SLAM integration in Week 3!**

---

## 🎉 Congratulations!

You've completed the setup phase. You now have:
- A configured development environment
- All necessary tools installed
- Hardware ready for integration
- A clear plan for the next 14 weeks

**Let's build this navigation system!**

---

**Next:** [Week 3-4: LiDAR SLAM Integration](PHASE1-PLAN.md#week-3-4-lidar-slam-integration)
