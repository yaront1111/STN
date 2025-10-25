# PX4/MAVLink Integration Specification

**Version:** 1.0.0
**Target:** PX4 v1.14+, MAVLink v2
**Purpose:** CHIMERA → PX4 external vision estimator

---

## Message Mapping

### 1. ODOMETRY (ID 331) - Primary State

**Publish Rate:** 100 Hz (from UKF propagation)

**Frame Convention:**
- `frame_id`: `MAV_FRAME_LOCAL_NED` (8)
- `child_frame_id`: `MAV_FRAME_BODY_FRD` (12)

**Message Structure:**

```cpp
void publishOdometry(const State& state) {
  mavlink_odometry_t msg;
  
  // Timestamp
  msg.time_usec = state.timestamp * 1e6;
  
  // Frame IDs
  msg.frame_id = MAV_FRAME_LOCAL_NED;
  msg.child_frame_id = MAV_FRAME_BODY_FRD;
  
  // Position (NED, meters)
  msg.x = state.pose.x();
  msg.y = state.pose.y();
  msg.z = state.pose.z();
  
  // Orientation (quaternion: w,x,y,z)
  Eigen::Quaterniond q(state.pose.rotation().matrix());
  msg.q[0] = q.w();
  msg.q[1] = q.x();
  msg.q[2] = q.y();
  msg.q[3] = q.z();
  
  // Velocity (NED, m/s)
  msg.vx = state.velocity.x();
  msg.vy = state.velocity.y();
  msg.vz = state.velocity.z();
  
  // Angular velocity (body frame, rad/s)
  msg.rollspeed = state.angular_velocity.x();
  msg.pitchspeed = state.angular_velocity.y();
  msg.yawspeed = state.angular_velocity.z();
  
  // Pose covariance (6x6: position + orientation)
  Eigen::Matrix<double, 6, 6> pose_cov = state.getPoseCovariance();
  for (int i = 0; i < 21; ++i) {  // Upper triangular
    int row = i / 6;
    int col = i % 6;
    if (col >= row) {
      msg.pose_covariance[i] = pose_cov(row, col);
    }
  }
  
  // Velocity covariance (6x6: linear + angular)
  Eigen::Matrix<double, 6, 6> vel_cov = state.getVelocityCovariance();
  for (int i = 0; i < 21; ++i) {
    int row = i / 6;
    int col = i % 6;
    if (col >= row) {
      msg.velocity_covariance[i] = vel_cov(row, col);
    }
  }
  
  mavlink_msg_odometry_send(...);
}
```

---

### 2. VISION_POSITION_ESTIMATE (ID 102) - Fallback

**Publish Rate:** 20 Hz
**Use:** If PX4 doesn't support ODOMETRY, use this simpler message

```cpp
void publishVisionPosition(const State& state) {
  mavlink_vision_position_estimate_t msg;
  
  msg.usec = state.timestamp * 1e6;
  msg.x = state.pose.x();
  msg.y = state.pose.y();
  msg.z = state.pose.z();
  
  // Euler angles (rad)
  auto rpy = state.pose.rotation().rpy();
  msg.roll = rpy.x();
  msg.pitch = rpy.y();
  msg.yaw = rpy.z();
  
  // Covariance (21 floats, row-major upper triangle)
  // [x, y, z, roll, pitch, yaw]
  // Fill msg.covariance[21]
  
  mavlink_msg_vision_position_estimate_send(...);
}
```

---

### 3. ATTITUDE (ID 30) - High-Rate Orientation

**Publish Rate:** 100 Hz
**Use:** Provide high-rate attitude for stabilization

```cpp
void publishAttitude(const State& state) {
  mavlink_attitude_t msg;
  
  msg.time_boot_ms = millis();
  
  auto rpy = state.pose.rotation().rpy();
  msg.roll = rpy.x();
  msg.pitch = rpy.y();
  msg.yaw = rpy.z();
  
  msg.rollspeed = state.angular_velocity.x();
  msg.pitchspeed = state.angular_velocity.y();
  msg.yawspeed = state.angular_velocity.z();
  
  mavlink_msg_attitude_send(...);
}
```

---

### 4. HEARTBEAT (ID 0) - System Status

**Publish Rate:** 1 Hz
**Purpose:** Indicate CHIMERA is alive

```cpp
void publishHeartbeat() {
  mavlink_heartbeat_t msg;
  
  msg.type = MAV_TYPE_ONBOARD_CONTROLLER;
  msg.autopilot = MAV_AUTOPILOT_INVALID;
  msg.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  msg.custom_mode = 0;
  msg.system_status = contract_ok_ ? MAV_STATE_ACTIVE : MAV_STATE_CRITICAL;
  
  mavlink_msg_heartbeat_send(...);
}
```

---

### 5. CHIMERA_STATUS (Custom, ID 13000) - Extended Diagnostics

**Publish Rate:** 1 Hz
**Purpose:** CHIMERA-specific health and mode info

```xml
<!-- chimera_status.xml -->
<message id="13000" name="CHIMERA_STATUS">
  <description>CHIMERA navigation system status</description>
  <field type="uint64_t" name="time_usec">Timestamp (microseconds)</field>
  <field type="float" name="altitude_agl">Altitude AGL (meters)</field>
  <field type="uint8_t" name="altitude_source">0=none, 1=lidar, 2=baro, 3=takeover</field>
  <field type="float" name="lidar_health">LiDAR health score [0,1]</field>
  <field type="float" name="baro_health">Baro health score [0,1]</field>
  <field type="float" name="mag_health">Mag health score [0,1]</field>
  <field type="float" name="of_health">Optical flow health score [0,1]</field>
  <field type="float" name="imu_health">IMU health score [0,1]</field>
  <field type="uint8_t" name="contract_ok">0=fail, 1=pass</field>
  <field type="uint32_t" name="mode_transitions">Count of altitude source switches</field>
  <field type="float" name="graph_error">Total factor graph error</field>
</message>
```

---

## PX4 Parameter Configuration

### EKF2 Settings (External Vision)

```bash
# Enable vision position + yaw
param set EKF2_AID_MASK 24

# Use vision for height
param set EKF2_HGT_MODE 3

# Vision delays (set to 0 for real-time)
param set EKF2_EV_DELAY 0

# Vision sensor position (body frame offsets, from calibration)
param set EKF2_EV_POS_X 0.0
param set EKF2_EV_POS_Y 0.0
param set EKF2_EV_POS_Z 0.0

# Vision quality gate (lower = more trust)
param set EKF2_EV_NOISE_MD 0

# Save
param save
```

### MAVLink Streaming

```bash
# Ensure PX4 receives ODOMETRY
mavlink stream -d /dev/ttyACM0 -s ODOMETRY -r 100

# Or in companion computer script
mavlink stream -u 14550 -s ODOMETRY -r 100
```

---

## Integration Architecture

```
┌─────────────────────┐
│  CHIMERA (Companion)│
│  - chimera_node     │
│  - MAVLink bridge   │
└──────────┬──────────┘
           │ MAVLink (serial/UDP)
           │ - ODOMETRY (100 Hz)
           │ - VISION_POS (20 Hz)
           │ - ATTITUDE (100 Hz)
           │ - HEARTBEAT (1 Hz)
           │ - CHIMERA_STATUS (1 Hz)
           ▼
┌─────────────────────┐
│  PX4 (Autopilot)    │
│  - EKF2             │
│  - Position Control │
└─────────────────────┘
```

---

## Failsafe Integration

### CHIMERA → PX4 Health Indication

**Method 1: HEARTBEAT system_status**
- `MAV_STATE_ACTIVE`: CHIMERA healthy (`contract_ok = true`)
- `MAV_STATE_CRITICAL`: CHIMERA degraded (`contract_ok = false`)

**Method 2: Stop publishing ODOMETRY**
- PX4 EKF2 will timeout after `EKF2_EV_DELAY` + margin
- Triggers vision loss failsafe

**Method 3: CHIMERA_STATUS.contract_ok**
- Custom field for fine-grained monitoring
- Ground station can display health

---

## Testing

### Verify MAVLink Connection

```bash
# On companion computer
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550

# Monitor messages
mavproxy> module load signing
mavproxy> ODOMETRY
```

### Check PX4 EKF2 Status

```bash
# On PX4 console
listener estimator_status

# Should show:
# - pos_horiz_accuracy: < 1.0 (good)
# - pos_vert_accuracy: < 0.5 (good)
# - innovation_check_flags: all pass
```

---

## Common Issues

### Issue: PX4 ignores ODOMETRY

**Cause:** EKF2_AID_MASK not set correctly

**Fix:**
```bash
param set EKF2_AID_MASK 24  # vision pos + yaw
param save
reboot
```

### Issue: Height estimate drifts

**Cause:** EKF2_HGT_MODE not using vision

**Fix:**
```bash
param set EKF2_HGT_MODE 3  # vision
param save
```

---

**See Also:**
- PX4 Docs: https://docs.px4.io/main/en/ros/external_position_estimation.html
- MAVLink Spec: https://mavlink.io/en/messages/common.html
