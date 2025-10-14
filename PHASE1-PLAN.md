# ORION-Lite: Phase 1 Implementation Plan
## Practical 16-Week GPS-Denied Navigation Development

**Based on Gemini Technical Consultation**
**Target:** 5-10 min flight, <5m drift, IMU + LiDAR SLAM

---

## Executive Summary

Following expert guidance, we are building **ORION-Lite** - a simplified but fully functional GPS-denied navigation system that serves as the foundation for the full ORION vision.

### Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Primary Sensor** | LiDAR + IMU | More robust than VIO, lighting-invariant |
| **Fusion Approach** | EKF (`robot_localization`) | Proven, fast to implement vs. GTSAM |
| **LiDAR SLAM** | KISS-ICP | Simple, robust, lightweight |
| **Compute** | Jetson Xavier NX | Sufficient for Phase 1, saves $200 |
| **Architecture** | Microservices (ROS2) | Modular, scalable, debuggable |
| **Development** | Hybrid sim + hardware | Best of both worlds |

### Success Criteria

- [ ] 5-10 minute autonomous GPS-denied flight
- [ ] <5m position drift over flight duration
- [ ] Stable position hold and waypoint navigation
- [ ] Measured performance with ground truth
- [ ] Full system documentation

---

## Hardware Bill of Materials

### Phase 1 "ORION-Lite" Kit (~$1,550)

| Component | Model | Cost | Specs | Notes |
|-----------|-------|------|-------|-------|
| **Compute** | Jetson Xavier NX Dev Kit | $399 | 21 TOPS, 8GB RAM | Upgrade to Orin in Phase 2 |
| **Stereo+IMU** | OAK-D Pro | $349 | Global shutter, 120fps, IMU | For Phase 2 VIO |
| **LiDAR** | Livox Mid-360 | $599 | 360° FOV, 200m range | Primary odometry sensor |
| **Flight Controller** | Pixhawk 4/6c | $250 | ARM Cortex-M7 | Industry standard |
| **Accessories** | Power, mounting, cables | ~$200 | Various | 5V/12V regulators, etc. |
| **Total** | | **~$1,797** | | |

**Postponed to Phase 2+:**
- VectorNav VN-300 tactical IMU ($3,500) - OAK-D Pro IMU sufficient for Phase 1
- Event camera ($5,000)
- Magnetometer array ($300)
- SDR ($350)

---

## Software Stack

| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| **OS** | Ubuntu 22.04 | LTS | ROS2 Humble base |
| **Middleware** | ROS2 Humble | Latest | Core framework |
| **LiDAR SLAM** | KISS-ICP | Latest | Primary odometry |
| **Fusion** | robot_localization | ROS2 port | EKF state estimation |
| **Flight Control** | PX4 | v1.14+ | Low-level stabilization |
| **Interface** | MAVROS | ROS2 version | ROS-PX4 bridge |
| **Simulation** | Gazebo | Fortress | Physics simulation |
| **Visualization** | RViz2 | Humble | Real-time debugging |

---

## System Architecture

### Node Graph

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ /imu_driver │────>│              │     │             │
└─────────────┘     │              │     │             │
                    │ /lidar_slam  │────>│ /ekf_fusion │────> /odom/filtered
┌──────────────┐    │  (KISS-ICP)  │     │  (robot_    │
│/lidar_driver │───>│              │     │localization)│
└──────────────┘    └──────────────┘     └─────────────┘
                                                 │
                                                 v
                                          ┌──────────────┐
                                          │   /mavros    │
                                          │ (PX4 bridge) │
                                          └──────────────┘
```

### Topics

| Topic | Type | Rate | Publisher | Subscriber |
|-------|------|------|-----------|------------|
| `/imu/data` | sensor_msgs/Imu | 400Hz | imu_driver | lidar_slam, ekf |
| `/points2` | sensor_msgs/PointCloud2 | 10Hz | lidar_driver | lidar_slam |
| `/odom/lidar` | nav_msgs/Odometry | 10Hz | lidar_slam | ekf |
| `/odom/filtered` | nav_msgs/Odometry | 100Hz | ekf | mavros, planner |

### TF Tree

```
map
 └── odom
      └── base_link
           ├── imu_link
           └── lidar_link
```

---

## 16-Week Implementation Timeline

### Month 1: Foundation & LiDAR SLAM (Weeks 1-4)

#### **Week 1-2: Setup & Hardware Bring-up**

**Goals:**
- All hardware operational and publishing data
- ROS2 workspace configured
- Version control and documentation started

**Tasks:**
- [ ] Order all hardware components
- [ ] Set up Ubuntu 22.04 on Jetson Xavier NX
- [ ] Install ROS2 Humble + dependencies
- [ ] Create Git repository with proper structure
- [ ] Write/install sensor drivers:
  - [ ] Livox Mid-360 driver (livox_ros_driver2)
  - [ ] OAK-D Pro driver (depthai-ros)
  - [ ] IMU driver (if separate from OAK-D)
- [ ] Verify data publishing: `ros2 topic list`, `ros2 topic echo`
- [ ] Record test rosbags of each sensor
- [ ] Create initial URDF with sensor transforms

**Deliverable:** All sensors publishing clean data, visualized in RViz2

---

#### **Week 3-4: LiDAR SLAM Integration**

**Goals:**
- KISS-ICP producing stable odometry
- Clean maps of test environment
- Parameter tuning complete

**Tasks:**
- [ ] Install KISS-ICP (https://github.com/PRBonn/kiss-icp)
- [ ] Configure KISS-ICP for Livox Mid-360 point cloud format
- [ ] Record rosbags walking drone around test area
- [ ] Run KISS-ICP offline on recorded data
- [ ] Tune parameters:
  - [ ] Voxel size
  - [ ] Max correspondence distance
  - [ ] Initial guess threshold
- [ ] Validate odometry by:
  - [ ] Observing map quality in RViz
  - [ ] Loop closure detection on repeated paths
  - [ ] Drift measurement (return to start position)
- [ ] Live testing: real-time KISS-ICP with live sensor data

**Deliverable:** KISS-ICP reliably producing <2% drift odometry on test rosbags

---

### Month 2: Fusion & Simulation (Weeks 5-8)

#### **Week 5-6: EKF Fusion**

**Goals:**
- Fused IMU + LiDAR odometry
- Smooth, high-rate state estimate (100Hz)
- Covariance tuning complete

**Tasks:**
- [ ] Install `robot_localization` package
- [ ] Create `ekf_localization_node` configuration YAML
- [ ] Configure input sources:
  - [ ] IMU: orientation, angular velocity, linear acceleration
  - [ ] LiDAR odometry: position, orientation
- [ ] Tune process and measurement covariances:
  - [ ] Start with defaults
  - [ ] Adjust based on sensor specifications
  - [ ] Iterate using test data
- [ ] Validate fusion:
  - [ ] Compare raw vs. filtered odometry in PlotJuggler
  - [ ] Verify covariance is reasonable (chi-squared test)
  - [ ] Test with sensor dropout scenarios
- [ ] Implement confidence estimator (covariance-based)

**Deliverable:** Smooth, filtered odometry at 100Hz, more stable than raw LiDAR odom

---

#### **Week 7-8: Flight Controller Integration & SITL**

**Goals:**
- Complete control loop working in simulation
- Stable position hold and waypoint navigation in Gazebo
- MAVROS communication validated

**Tasks:**
- [ ] Install PX4 Autopilot + Gazebo sim
- [ ] Set up PX4 SITL (Software-In-The-Loop)
- [ ] Install and configure MAVROS
- [ ] Create simulated UAV model with LiDAR + IMU
- [ ] Connect filtered odometry to PX4:
  - [ ] Publish vision position estimate to MAVROS
  - [ ] Configure PX4 parameters for vision-based nav
- [ ] Test basic flight modes in sim:
  - [ ] Position hold
  - [ ] Waypoint navigation (simple square pattern)
  - [ ] Return to home
- [ ] Tune PX4 position controller gains
- [ ] Implement safety checks (e.g., drift threshold triggers RTL)

**Deliverable:** Simulated UAV flying stable 5-min missions in Gazebo using LiDAR+IMU nav

---

### Month 3: Hardware Integration & First Flights (Weeks 9-12)

#### **Week 9-10: Hardware-in-the-Loop Integration**

**Goals:**
- All hardware mounted on UAV frame
- Complete system running on real hardware
- Calibration complete

**Tasks:**
- [ ] Design and fabricate/3D-print sensor mounts
- [ ] Mount Jetson, sensors, flight controller on UAV frame
- [ ] Wire power distribution (5V for Jetson, 12V for Livox, etc.)
- [ ] Set up cooling for Jetson (heatsink + fan)
- [ ] Perform sensor calibration:
  - [ ] IMU calibration (6-point tumble)
  - [ ] LiDAR-IMU extrinsic calibration
  - [ ] Camera intrinsic/extrinsic calibration (for future VIO)
- [ ] Build complete TF tree with measured transforms
- [ ] Verify system integration:
  - [ ] All nodes launching correctly
  - [ ] Data flowing through full pipeline
  - [ ] Filtered odometry stable on ground
- [ ] Implement vibration damping (foam/rubber mounts)
- [ ] Test communication range (WiFi telemetry)

**Deliverable:** Fully integrated UAV with complete nav stack running, ready for flight

---

#### **Week 11-12: First Flight Tests**

**Goals:**
- Safe hover and basic maneuvers
- Validation of navigation system in real flight
- Issue identification and resolution

**Tasks:**
- [ ] Pre-flight checklist creation
- [ ] Safety procedures and emergency protocols
- [ ] Tethered hover tests (3-5 min):
  - [ ] Verify position hold stability
  - [ ] Monitor odometry drift
  - [ ] Check for vibration-induced issues
  - [ ] Tune flight controller gains if needed
- [ ] Short untethered flights (<2 min):
  - [ ] Indoor or protected outdoor area
  - [ ] Simple waypoint missions (5m square)
  - [ ] Land and check drift vs. takeoff position
- [ ] Debug real-world issues:
  - [ ] Motion blur in camera (for future VIO)
  - [ ] IMU noise from vibration
  - [ ] LiDAR ground reflection issues
  - [ ] WiFi dropouts
- [ ] Iterate on mechanical and software fixes

**Deliverable:** Demonstrated stable 5-minute GPS-denied hover with <2m drift

---

### Month 4: Field Testing & Demo Preparation (Weeks 13-16)

#### **Week 13-14: Performance Validation**

**Goals:**
- Quantified navigation performance
- Ground truth comparison
- Identified failure modes

**Tasks:**
- [ ] Define test scenarios:
  - [ ] Hover in place (10 min)
  - [ ] Square waypoint pattern (20m sides)
  - [ ] Figure-8 pattern
  - [ ] Altitude changes (10m climb/descent)
- [ ] Set up ground truth measurement:
  - [ ] Surveyed markers in test area
  - [ ] RTK GNSS for outdoor comparison
  - [ ] Or hand-measured returns to known positions
- [ ] Conduct structured test flights (5+ per scenario)
- [ ] Record rosbags of all flights
- [ ] Analyze performance:
  - [ ] Plot odometry vs. ground truth
  - [ ] Calculate drift metrics (RMS error, max error)
  - [ ] Identify when/why failures occur
- [ ] Failure mode analysis:
  - [ ] What happens if LiDAR loses tracking?
  - [ ] How does vibration affect performance?
  - [ ] Wind gust response
- [ ] Implement mitigations for identified issues

**Deliverable:** Performance report showing <5m drift over 10-min flights

---

#### **Week 15-16: Refinement & Demo**

**Goals:**
- Polished, reliable demo
- Complete documentation
- Roadmap for Phase 2

**Tasks:**
- [ ] Final system tuning based on test results
- [ ] Streamline launch procedures
- [ ] Create demonstration mission:
  - [ ] 10-minute GPS-denied flight
  - [ ] Waypoint navigation
  - [ ] Return to launch with <5m error
  - [ ] Live visualization (RViz + telemetry)
- [ ] Documentation:
  - [ ] System architecture diagram
  - [ ] Hardware setup guide
  - [ ] Software installation/build instructions
  - [ ] Calibration procedures
  - [ ] Launch file documentation
  - [ ] Troubleshooting guide
- [ ] Record demo video (screen capture + external camera)
- [ ] Prepare Phase 2 proposal:
  - [ ] Add VIO (VINS-Fusion)
  - [ ] Transition to GTSAM factor graph
  - [ ] Loop closure integration
  - [ ] Extend flight time to 20-30 min

**Deliverable:** Complete "ORION-Lite" system with demo and documentation

---

## Risk Management

### Risk Matrix

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| **Hardware delays** | High | High | Order early, have backup suppliers |
| **Vibration issues** | Medium | High | Proper damping, IMU filtering |
| **LiDAR tracking loss** | Medium | High | Tune KISS-ICP, ensure good environment |
| **EKF instability** | Medium | Medium | Conservative tuning, extensive testing |
| **PX4 integration** | Low | High | Use SITL first, follow examples |
| **Power budget** | Low | Medium | Careful power planning, bench tests |
| **Calibration errors** | Medium | High | Use proven tools, validate thoroughly |

### De-Risking Strategies

1. **Incremental Integration:** Test each component individually before full system
2. **Simulation First:** Validate control loops in SITL before real flights
3. **Redundant Testing:** Always have backup rosbags for offline development
4. **Community Support:** Use proven, well-documented packages
5. **Safety Margins:** Conservative flight tests, gradual complexity increase

---

## What's Postponed to Phase 2

**Technology Deferred:**
- GTSAM factor graph fusion → EKF sufficient for <5m drift
- Stereo VIO integration → LiDAR-only is simpler and more robust initially
- Vision Transformer terrain matching → Research-level, Phase 3+
- Loop closure detection → Not critical for 10-min flights
- Confidence-based mode switching → Basic covariance check sufficient

**Hardware Deferred:**
- Tactical-grade IMU (VectorNav VN-300)
- Event camera
- Magnetometer array
- SDR for RF navigation
- Jetson Orin NX (Xavier NX sufficient)

**These will be added incrementally in Phase 2-3 once we have a working baseline.**

---

## Success Metrics

### Quantitative

- [ ] Position drift: <5m after 10-min flight
- [ ] Update rate: 100Hz filtered odometry
- [ ] Latency: <50ms end-to-end
- [ ] Success rate: >80% of test flights complete mission
- [ ] Recovery: <10s for system restart after ground landing

### Qualitative

- [ ] Stable position hold in hover
- [ ] Smooth waypoint navigation
- [ ] Graceful behavior on tracking loss (e.g., RTL)
- [ ] Predictable and debuggable system
- [ ] Maintainable and extensible codebase

---

## Phase 2 Preview

Once Phase 1 "ORION-Lite" is complete, Phase 2 will add:

1. **Stereo VIO (VINS-Fusion)** as second odometry source
2. **GTSAM factor graph** to replace EKF for multi-modal fusion
3. **Loop closure** for long-duration flight (30+ min)
4. **Basic terrain matching** using classical features (not ViT yet)
5. **Extended flight time** to 20-30 minutes
6. **Target accuracy** improved to <2m

**Phase 2 Timeline:** 3-4 months (Weeks 17-32)

---

## Resources

### Key Documentation

- [KISS-ICP GitHub](https://github.com/PRBonn/kiss-icp)
- [robot_localization Docs](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [PX4 User Guide](https://docs.px4.io/)
- [MAVROS](https://github.com/mavlink/mavros)
- [Livox SDK](https://github.com/Livox-SDK/livox_ros_driver2)

### Learning Resources

- ROS2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- SLAM Course: https://www.youtube.com/watch?v=U6vr3iNrwRA (Cyrill Stachniss)
- Kalman Filter: https://www.kalmanfilter.net/
- PX4 Development: https://dev.px4.io/

---

## Conclusion

By following this **ORION-Lite** plan, we will achieve a working GPS-denied navigation system in 4 months. This is a realistic, de-risked path that proves the core concept and builds a solid foundation for the full ORION vision.

**Remember:** The goal of Phase 1 is not perfection—it's a working system that flies. Once we have that, we can iterate and enhance.

**Let's build this step by step, and get that UAV flying without GPS!**
