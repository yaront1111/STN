# Integration & UX Specifications

**Track:** P0 (Critical for Deployment)
**Owner:** Integration Team
**Timeline:** Weeks 4-6
**Primary Focus:** ROS2 Ecosystem

---

## Overview

Enable CHIMERA deployment across three integration points:
1. **ROS2 Node** (Primary) - Full ecosystem integration
2. **PX4/MAVLink Bridge** - Autopilot compatibility
3. **C++ SDK** - Direct integration for custom systems

---

## 1. ROS2 Node (Primary Integration)

### Package Structure

```
chimera_nav/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── chimera.launch.py
│   ├── sensors.launch.py
│   └── rviz.launch.py
├── config/
│   ├── params.yaml
│   ├── sensors/
│   │   ├── imu.yaml
│   │   ├── lidar.yaml
│   │   ├── baro.yaml
│   │   ├── mag.yaml
│   │   └── optical_flow.yaml
│   └── rviz/
│       └── chimera.rviz
├── src/
│   ├── chimera_node.cpp
│   └── sensor_adapters/
│       ├── imu_adapter.cpp
│       ├── lidar_adapter.cpp
│       └── ...
├── include/chimera_nav/
│   └── chimera_node.hpp
└── README.md
```

### Topics (ROS2 Standard Messages)

#### Published

```yaml
/chimera/odometry:
  type: nav_msgs/Odometry
  qos: RELIABLE, depth=10
  rate: 100 Hz (from UKF propagation)
  frame_id: "odom"
  child_frame_id: "base_link"
  content:
    pose: 6-DOF pose (position + orientation)
    twist: 6-DOF velocity (linear + angular)
    covariance: 6x6 pose/twist uncertainty

/chimera/path:
  type: nav_msgs/Path
  qos: RELIABLE, depth=1
  rate: 5 Hz
  frame_id: "odom"
  content: Trajectory history (last N poses)

/chimera/diagnostics:
  type: diagnostic_msgs/DiagnosticArray
  qos: RELIABLE, depth=1
  rate: 1 Hz
  content:
    - Sensor health scores
    - Altitude source (lidar/baro/takeover)
    - Mode transitions
    - Processing deadlines
    - Resource usage

/chimera/altitude:
  type: sensor_msgs/Range
  qos: RELIABLE, depth=10
  rate: Variable (sensor-dependent)
  frame_id: "base_link"
  content: Altitude AGL with source metadata

/chimera/health:
  type: chimera_msgs/SystemHealth (custom)
  qos: RELIABLE, depth=1
  rate: 1 Hz
  content:
    lidar_health: {score, is_stuck, noise, reason}
    baro_health: {score, drift_rate, reason}
    mag_health: {score, interference, reason}
    of_health: {score, texture, reason}
    imu_health: {score, temperature, reason}
```

#### Subscribed

```yaml
/imu/data:
  type: sensor_msgs/Imu
  qos: BEST_EFFORT, depth=100
  rate: 400 Hz
  
/lidar/range:
  type: sensor_msgs/Range
  qos: RELIABLE, depth=10
  rate: Variable
  
/baro/pressure:
  type: sensor_msgs/FluidPressure
  qos: RELIABLE, depth=10
  rate: 20 Hz
  
/mag/field:
  type: sensor_msgs/MagneticField
  qos: RELIABLE, depth=10
  rate: 20 Hz
  
/camera/optical_flow:
  type: chimera_msgs/OpticalFlow (custom)
  qos: RELIABLE, depth=10
  rate: Variable
```

### Parameters

```yaml
chimera_nav:
  ros__parameters:
    # Sensor configuration
    imu_topic: "/imu/data"
    lidar_topic: "/lidar/range"
    baro_topic: "/baro/pressure"
    mag_topic: "/mag/field"
    optical_flow_topic: "/camera/optical_flow"
    
    # Frame IDs
    world_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    
    # Estimator parameters
    smoother_hz: 5.0
    lag_seconds: 15.0
    gravity: 9.81
    
    # Noise models
    gyro_noise_density: 0.0001
    accel_noise_density: 0.001
    lidar_sigma_base: 0.05
    baro_sigma_base: 0.3
    mag_sigma_deg: 5.0
    
    # Thresholds
    mag_gate_frac: 0.30
    lidar_stuck_eps: 0.05
    baro_takeover_thresh_m: 50.0
    
    # Publishing rates
    odom_rate: 100.0
    diagnostics_rate: 1.0
```

### TF Tree

```
map (fixed world frame)
 └─> odom (drifting odometry frame, published by chimera_nav)
      └─> base_link (robot body frame)
           ├─> imu_link
           ├─> lidar_link
           ├─> camera_link
           └─> mag_link
```

**Broadcast:**
- `odom → base_link`: CHIMERA estimates (100 Hz)
- `base_link → sensor_links`: Static transforms (from calibration)

### Lifecycle Node

```cpp
class ChimeraNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  CallbackReturn on_configure(const State&) override {
    // Load parameters, validate calibration
    loadParameters();
    if (!loadCalibration()) return FAILURE;
    
    // Create publishers/subscribers
    setupPublishers();
    setupSubscribers();
    
    return SUCCESS;
  }

  CallbackReturn on_activate(const State&) override {
    // Initialize CHIMERA core
    flight_computer_.init(sensors_);
    
    // Start timers
    odom_timer_ = create_wall_timer(10ms, &ChimeraNode::publishOdom);
    diag_timer_ = create_wall_timer(1s, &ChimeraNode::publishDiagnostics);
    
    return SUCCESS;
  }

  CallbackReturn on_deactivate(const State&) override {
    odom_timer_->cancel();
    diag_timer_->cancel();
    return SUCCESS;
  }

  CallbackReturn on_cleanup(const State&) override {
    // Release resources
    return SUCCESS;
  }

 private:
  FlightComputer flight_computer_;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::Imu>::SharedPtr imu_sub_;
  // ...
};
```

### nav2 Integration

```yaml
# nav2_params.yaml
global_costmap:
  update_frequency: 5.0
  publish_frequency: 2.0
  global_frame: odom  # Use CHIMERA odom frame
  robot_base_frame: base_link
  
local_costmap:
  update_frequency: 10.0
  publish_frequency: 5.0
  global_frame: odom
  robot_base_frame: base_link
  
controller:
  odom_topic: "/chimera/odometry"  # Use CHIMERA odometry
```

---

## 2. PX4/MAVLink Bridge

### MAVLink Messages

#### ODOMETRY (ID 331)

```cpp
void publishOdometry(const State& state) {
  mavlink_odometry_t msg;
  msg.time_usec = state.timestamp * 1e6;
  msg.frame_id = MAV_FRAME_LOCAL_NED;
  msg.child_frame_id = MAV_FRAME_BODY_FRD;
  
  // Position (NED)
  msg.x = state.pose.x();
  msg.y = state.pose.y();
  msg.z = state.pose.z();
  
  // Orientation (quaternion)
  Eigen::Quaterniond q(state.pose.rotation().matrix());
  msg.q[0] = q.w();
  msg.q[1] = q.x();
  msg.q[2] = q.y();
  msg.q[3] = q.z();
  
  // Velocity (NED)
  msg.vx = state.velocity.x();
  msg.vy = state.velocity.y();
  msg.vz = state.velocity.z();
  
  // Covariance (pose: 6x6, velocity: 6x6)
  // Fill msg.pose_covariance, msg.velocity_covariance
  
  mavlink_send(&msg);
}
```

#### VISION_POSITION_ESTIMATE (ID 102)

```cpp
void publishVisionPosition(const State& state) {
  mavlink_vision_position_estimate_t msg;
  msg.usec = state.timestamp * 1e6;
  msg.x = state.pose.x();
  msg.y = state.pose.y();
  msg.z = state.pose.z();
  
  // Roll, pitch, yaw
  auto rpy = state.pose.rotation().rpy();
  msg.roll = rpy.x();
  msg.pitch = rpy.y();
  msg.yaw = rpy.z();
  
  // Covariance (6-DOF pose)
  // Fill msg.covariance
  
  mavlink_send(&msg);
}
```

#### ATTITUDE (ID 30)

```cpp
void publishAttitude(const State& state) {
  mavlink_attitude_t msg;
  msg.time_boot_ms = millis();
  
  auto rpy = state.pose.rotation().rpy();
  msg.roll = rpy.x();
  msg.pitch = rpy.y();
  msg.yaw = rpy.z();
  
  // Angular rates (from UKF state derivative)
  msg.rollspeed = state.angular_velocity.x();
  msg.pitchspeed = state.angular_velocity.y();
  msg.yawspeed = state.angular_velocity.z();
  
  mavlink_send(&msg);
}
```

#### Custom: CHIMERA_STATUS (ID 13000)

```cpp
struct mavlink_chimera_status_t {
  uint64_t time_usec;
  float altitude_agl;
  uint8_t altitude_source;  // 0=none, 1=lidar, 2=baro, 3=takeover
  float lidar_health;
  float baro_health;
  float mag_health;
  float of_health;
  float imu_health;
  uint8_t contract_ok;
};
```

### PX4 Parameter Mapping

```yaml
# CHIMERA → PX4 EKF settings
EKF2_AID_MASK: 24  # vision position + vision yaw
EKF2_HGT_MODE: 3   # vision (use CHIMERA altitude)
EKF2_EV_DELAY: 0   # No delay (real-time)
EKF2_EV_POS_X: 0.0  # Vision sensor position (from calibration)
EKF2_EV_POS_Y: 0.0
EKF2_EV_POS_Z: 0.0
```

---

## 3. C++ SDK

### Stable API (v1.0.0)

```cpp
namespace chimera {

// Error codes
enum class Status {
  OK = 0,
  INVALID_CONFIG = 1,
  CALIBRATION_FAILED = 2,
  SENSOR_TIMEOUT = 3,
  DIVERGENCE_DETECTED = 4,
  RESOURCE_EXCEEDED = 5
};

// Main interface
class NavigationSystem {
 public:
  // Initialize with config
  Status initialize(const Config& config);
  
  // Add sensor measurements
  Status addImu(const ImuSample& sample);
  Status addLidar(const LidarSample& sample);
  Status addBaro(const BaroSample& sample);
  Status addMag(const MagSample& sample);
  Status addOpticalFlow(const OpticalFlowSample& sample);
  
  // Get current state estimate
  State getState() const;
  Health getHealth() const;
  
  // Register callbacks
  void setStateCallback(std::function<void(const State&)> cb);
  void setHealthCallback(std::function<void(const Health&)> cb);
  
  // Lifecycle
  void start();
  void stop();
  bool isRunning() const;
  
  // Version
  static std::string version();  // Returns "1.0.0"
};

// Semantic versioning
struct Version {
  uint32_t major = 1;
  uint32_t minor = 0;
  uint32_t patch = 0;
  std::string toString() const;
};

} // namespace chimera
```

### Config Schema (Typed & Validated)

```cpp
struct Config {
  struct Estimator {
    double smoother_hz = 5.0;
    double lag_seconds = 15.0;
    double gravity = 9.81;
  } estimator;
  
  struct Sensors {
    double gyro_noise_density = 0.0001;
    double accel_noise_density = 0.001;
    double lidar_sigma_base = 0.05;
    double baro_sigma_base = 0.3;
    double mag_sigma_deg = 5.0;
  } sensors;
  
  struct Thresholds {
    double mag_gate_frac = 0.30;
    double lidar_stuck_eps = 0.05;
    double baro_takeover_thresh_m = 50.0;
  } thresholds;
  
  // Validation
  Status validate() const;
  
  // Serialization
  static Config loadFromFile(const std::string& path);
  void saveToFile(const std::string& path) const;
};
```

---

## Implementation Checklist

### Week 4
- [ ] ROS2 package skeleton (CMakeLists, package.xml)
- [ ] Topic publishers (odometry, diagnostics)
- [ ] Lifecycle node structure
- [ ] Parameter loading

### Week 5
- [ ] Sensor subscribers (IMU, LiDAR, Baro, Mag, OF)
- [ ] TF broadcaster (odom→base_link)
- [ ] MAVLink bridge (ODOMETRY, VISION_POSITION_ESTIMATE)
- [ ] Custom messages (chimera_msgs)

### Week 6
- [ ] Launch files (single/multi-sensor configs)
- [ ] nav2 integration testing
- [ ] PX4 integration validation
- [ ] C++ SDK header finalization

---

## Acceptance Criteria

- [ ] ROS2 node publishes odometry at 100 Hz
- [ ] TF tree correct (map→odom→base_link)
- [ ] nav2 can use CHIMERA for navigation
- [ ] PX4 EKF2 accepts CHIMERA vision input
- [ ] All parameters load from YAML
- [ ] Lifecycle transitions work correctly

---

**See Also:**
- [ros2_architecture.md](../specs/ros2_architecture.md) - Detailed ROS2 design
- [mavlink_mapping.md](../specs/mavlink_mapping.md) - Complete MAVLink spec
