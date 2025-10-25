# Robustness & Reliability Specifications

**Track:** P0 (Critical for Production)
**Owner:** Core Estimation Team
**Timeline:** Weeks 1-9
**Dependencies:** None (foundation track)

---

## Overview

Robustness transforms CHIMERA from a research demo (11.89m accuracy on clean data) to a production system that handles real-world chaos: clock skew, sensor failures, thermal drift, resource limits, and numerical edge cases.

**Goals:**
1. **Fault Tolerance**: Detect and gracefully degrade when sensors fail
2. **Determinism**: Predictable behavior under all conditions
3. **Resource Safety**: Never exceed CPU/memory budgets
4. **Numerical Stability**: No NaNs, infinities, or covariance blow-ups

---

## 1. Time Synchronization & Clock Drift

### Problem
- Sensors have independent clocks (IMU @ 400Hz, LiDAR @ variable, Baro @ 20Hz)
- System clock may drift or jump (NTP corrections, suspend/resume)
- Out-of-order timestamps corrupt factor graph

### Architecture

#### Monotonic Timebase
```cpp
class MonotonicClock {
 public:
  // Initialize with system time or external sync
  void init(double t0_system);

  // Convert system timestamp to monotonic time
  double toMonotonic(double t_system) const;

  // Detect and handle clock jumps
  void handleJump(double t_old, double t_new);

 private:
  double t0_mono_;
  double last_system_time_;
  double skew_rate_;  // Estimated drift (ppm)
};
```

#### Skew Estimation
```cpp
class ClockSkewEstimator {
 public:
  // Update with external time reference (e.g., GNSS when available)
  void update(double t_local, double t_reference);

  // Get current skew estimate (ppm)
  double getSkewPPM() const;

  // Predict monotonic time from system time
  double predict(double t_system) const;

 private:
  // Linear regression: t_ref = a * t_local + b
  // Skew = (a - 1) * 1e6 ppm
  double a_, b_;
  std::deque<std::pair<double, double>> history_;
};
```

#### Out-of-Order Sample Handling
```cpp
struct SensorSample {
  double t_system;
  double t_mono;  // Converted timestamp
  ...
};

// In sensor processing
if (sample.t_mono < last_processed_time_) {
  LOG_WARN("Out-of-order sample: t=" << sample.t_mono
           << ", last=" << last_processed_time_);

  if (sample.t_mono < last_processed_time_ - tolerance_) {
    // Too old, drop
    stats_.dropped_late_samples++;
    return;
  } else {
    // Within tolerance, flag and insert at correct position
    sample.flags |= SampleFlags::OUT_OF_ORDER;
    insertSorted(buffer_, sample);
  }
}
```

#### Telemetry
```cpp
struct ClockHealth {
  double skew_ppm;          // Estimated drift
  double last_jump_sec;     // Magnitude of last jump
  uint64_t jumps_detected;  // Total jumps since boot
  uint64_t ooo_samples;     // Out-of-order samples
};
```

---

## 2. Per-Sensor Health Monitoring

### Goal
Each sensor gets a **health score [0.0, 1.0]** based on:
- **Quality**: Signal characteristics (noise, saturation, validity)
- **Reliability**: Recent failure rate, recovery behavior
- **Consistency**: Agreement with other sensors

### LiDAR Health

```cpp
class LidarHealthMonitor {
 public:
  struct Health {
    double score;         // [0,1]
    bool is_stuck;        // Plateau detected
    bool out_of_range;    // > max_range or < min_range
    double noise_level;   // Stddev over window
    std::string reason;   // Human-readable
  };

  Health evaluate(const LidarSample& sample) {
    Health h;
    h.score = 1.0;

    // Check range validity
    if (sample.range_mean < min_range_ || sample.range_mean > max_range_) {
      h.out_of_range = true;
      h.score *= 0.1;
      h.reason += "out_of_range ";
    }

    // Stuck detection (existing logic)
    plateau_guard_.push(sample.range_mean);
    if (plateau_guard_.isStuck(stuck_eps_)) {
      h.is_stuck = true;
      h.score = 0.0;
      h.reason += "stuck_plateau ";
    }

    // Noise level
    h.noise_level = computeNoise(sample);
    if (h.noise_level > noise_thresh_) {
      h.score *= 0.5;
      h.reason += "high_noise ";
    }

    return h;
  }

 private:
  double computeNoise(const LidarSample& s) {
    // Use range_max - range_min as proxy
    return s.range_max - s.range_min;
  }

  PlateauGuard plateau_guard_;
  double min_range_ = 0.3;  // meters
  double max_range_ = 50.0;
  double stuck_eps_ = 0.05;
  double noise_thresh_ = 0.5;
};
```

### Baro Health

```cpp
class BaroHealthMonitor {
 public:
  struct Health {
    double score;
    double noise_level;    // Stddev of recent samples
    double drift_rate;     // m/s (OU bias derivative)
    bool rapid_change;     // Step > threshold
    std::string reason;
  };

  Health evaluate(const BaroSample& sample) {
    Health h;
    h.score = 1.0;

    // Update noise estimator
    noise_window_.push(sample.altitude_agl);
    h.noise_level = noise_window_.stddev();

    if (h.noise_level > noise_thresh_) {
      h.score *= 0.7;
      h.reason += "high_noise ";
    }

    // Detect rapid changes (pressure step)
    if (!history_.empty()) {
      double dz = sample.altitude_agl - history_.back().altitude_agl;
      double dt = sample.t - history_.back().t;
      double rate = std::abs(dz / dt);

      if (rate > rapid_change_thresh_) {
        h.rapid_change = true;
        h.score *= 0.3;
        h.reason += "rapid_change ";
      }
    }

    // Track drift rate (from OU bias filter)
    h.drift_rate = estimateDriftRate();

    history_.push_back(sample);
    if (history_.size() > max_history_) history_.pop_front();

    return h;
  }

 private:
  RollingWindow noise_window_{20};  // 1 second @ 20 Hz
  std::deque<BaroSample> history_;
  size_t max_history_ = 100;
  double noise_thresh_ = 0.5;  // meters stddev
  double rapid_change_thresh_ = 5.0;  // m/s
};
```

### Mag Health

```cpp
class MagHealthMonitor {
 public:
  struct Health {
    double score;
    double norm;           // |m|
    double norm_deviation; // |norm - median_norm|
    bool interference;     // Spike or norm out of gate
    std::string reason;
  };

  Health evaluate(const MagSample& sample) {
    Health h;
    h.score = 1.0;
    h.norm = sample.mag.norm();

    // Norm gating (existing logic)
    h.norm_deviation = std::abs(h.norm - mag_median_norm_);
    double gate_abs = mag_median_norm_ * mag_gate_frac_;

    if (h.norm_deviation > gate_abs) {
      h.interference = true;
      h.score = 0.0;
      h.reason += "norm_gate ";
    }

    // Spike detection
    if (!history_.empty()) {
      double dnorm = std::abs(h.norm - history_.back().norm);
      if (dnorm > spike_thresh_) {
        h.interference = true;
        h.score *= 0.2;
        h.reason += "spike ";
      }
    }

    history_.push_back({sample.t, h.norm});
    if (history_.size() > max_history_) history_.pop_front();

    return h;
  }

 private:
  double mag_median_norm_ = 1.0;  // From boot
  double mag_gate_frac_ = 0.3;
  double spike_thresh_ = 0.2;  // Norm change threshold
  std::deque<std::pair<double, double>> history_;
  size_t max_history_ = 50;
};
```

### Optical Flow Health

```cpp
class OpticalFlowHealthMonitor {
 public:
  struct Health {
    double score;
    double texture_quality;  // [0,1] from camera
    bool saturated;          // Flow magnitude too high
    bool low_texture;        // Texture below threshold
    uint32_t rejected_rays;  // Current frame
    std::string reason;
  };

  Health evaluate(const OpticalFlowSample& sample) {
    Health h;
    h.score = 1.0;
    h.texture_quality = sample.quality;  // From OF algorithm

    // Texture gating
    if (h.texture_quality < texture_thresh_) {
      h.low_texture = true;
      h.score *= h.texture_quality;  // Proportional penalty
      h.reason += "low_texture ";
    }

    // Saturation check
    double flow_mag = std::sqrt(sample.u_dot*sample.u_dot +
                                 sample.v_dot*sample.v_dot);
    if (flow_mag > saturation_thresh_) {
      h.saturated = true;
      h.score *= 0.1;
      h.reason += "saturated ";
    }

    return h;
  }

 private:
  double texture_thresh_ = 0.3;
  double saturation_thresh_ = 200.0;  // px/s
};
```

### IMU Health

```cpp
class ImuHealthMonitor {
 public:
  struct Health {
    double score;
    bool gyro_saturated;
    bool accel_saturated;
    double temperature;
    bool temp_out_of_range;
    std::string reason;
  };

  Health evaluate(const ImuSample& sample) {
    Health h;
    h.score = 1.0;
    h.temperature = sample.temperature;

    // Saturation checks
    if (sample.gyro.cwiseAbs().maxCoeff() > gyro_sat_thresh_) {
      h.gyro_saturated = true;
      h.score = 0.0;
      h.reason += "gyro_sat ";
    }

    if (sample.accel.cwiseAbs().maxCoeff() > accel_sat_thresh_) {
      h.accel_saturated = true;
      h.score = 0.0;
      h.reason += "accel_sat ";
    }

    // Temperature range
    if (h.temperature < temp_min_ || h.temperature > temp_max_) {
      h.temp_out_of_range = true;
      h.score *= 0.8;
      h.reason += "temp_range ";
    }

    return h;
  }

 private:
  double gyro_sat_thresh_ = 8.0;    // rad/s (~460 deg/s)
  double accel_sat_thresh_ = 150.0; // m/s^2 (~15g)
  double temp_min_ = -10.0;         // Celsius
  double temp_max_ = 60.0;
};
```

---

## 3. Deterministic Mode Switching

### Altitude Source Selection

```cpp
enum class AltitudeSource {
  NONE,
  LIDAR,
  BARO,
  BARO_TAKEOVER
};

class AltitudeSourceSelector {
 public:
  AltitudeSource select(
      const LidarHealthMonitor::Health& lidar_h,
      const BaroHealthMonitor::Health& baro_h,
      const State& current_state) {

    // Priority: LiDAR > Baro > Takeover

    // LiDAR preferred if healthy
    if (lidar_h.score > 0.5 && !lidar_h.is_stuck) {
      return AltitudeSource::LIDAR;
    }

    // Check for divergence (trigger takeover)
    double z_pred = current_state.pose.z();
    double z_baro = -baro_h.latest_agl;  // Convert to nav frame

    if (std::abs(z_pred - z_baro) > takeover_thresh_) {
      LOG_WARN("Altitude divergence: pred=" << z_pred
               << " baro=" << z_baro << " → BARO_TAKEOVER");
      return AltitudeSource::BARO_TAKEOVER;
    }

    // Fall back to baro if healthy
    if (baro_h.score > 0.3) {
      return AltitudeSource::BARO;
    }

    // No good altitude source!
    LOG_ERROR("No healthy altitude source!");
    return AltitudeSource::NONE;
  }

 private:
  double takeover_thresh_ = 50.0;  // meters
};
```

### Mode Transition Telemetry

```cpp
struct ModeTransition {
  double timestamp;
  AltitudeSource from;
  AltitudeSource to;
  std::string reason;
};

// Log transitions
if (new_source != current_source_) {
  ModeTransition trans{t, current_source_, new_source, reason};
  mode_transitions_.push_back(trans);

  // Telemetry
  telemetry_.log("alt_source_change", {
    {"t", t},
    {"from", toString(current_source_)},
    {"to", toString(new_source)},
    {"reason", reason}
  });

  current_source_ = new_source;
}
```

---

## 4. Watchdogs & Failsafes

### Processing Deadline Monitors

```cpp
class DeadlineMonitor {
 public:
  void start(const std::string& name, double deadline_ms) {
    auto& ctx = contexts_[name];
    ctx.start_time = steady_clock::now();
    ctx.deadline_ms = deadline_ms;
  }

  void end(const std::string& name) {
    auto& ctx = contexts_[name];
    auto elapsed_ms = duration_cast<milliseconds>(
        steady_clock::now() - ctx.start_time).count();

    ctx.last_duration_ms = elapsed_ms;
    ctx.max_duration_ms = std::max(ctx.max_duration_ms, (double)elapsed_ms);

    if (elapsed_ms > ctx.deadline_ms) {
      ctx.violations++;
      LOG_WARN(name << " deadline violated: " << elapsed_ms
               << "ms > " << ctx.deadline_ms << "ms");
    }
  }

  struct Stats {
    double last_duration_ms;
    double max_duration_ms;
    uint64_t violations;
  };

  Stats getStats(const std::string& name) const {
    return contexts_.at(name);
  }

 private:
  struct Context {
    time_point start_time;
    double deadline_ms;
    double last_duration_ms = 0.0;
    double max_duration_ms = 0.0;
    uint64_t violations = 0;
  };

  std::map<std::string, Context> contexts_;
};

// Usage
DeadlineMonitor monitor;

// In UKF loop
monitor.start("ukf_propagate", 0.5);  // 0.5ms deadline
ukf_.propagate(imu_sample);
monitor.end("ukf_propagate");

// In smoother loop
monitor.start("smoother_update", 30.0);  // 30ms deadline
smoother_.update(graph, init);
monitor.end("smoother_update");
```

### NaN Guards

```cpp
template<typename T>
bool hasNaN(const T& value) {
  return !std::isfinite(value);
}

template<>
bool hasNaN(const Eigen::VectorXd& v) {
  return !v.allFinite();
}

template<>
bool hasNaN(const gtsam::Pose3& pose) {
  return hasNaN(pose.translation()) || hasNaN(pose.rotation().matrix());
}

// In critical paths
gtsam::Vector3 velocity = result.at<gtsam::Vector3>(V(key));
if (hasNaN(velocity)) {
  LOG_ERROR("NaN detected in velocity at key=" << key);
  // Failsafe: revert to previous state
  velocity = last_good_velocity_;
  telemetry_.log("nan_detected", {{"variable", "velocity"}, {"key", key}});
}
```

### Covariance Blow-up Clamps

```cpp
void clampCovariance(Eigen::MatrixXd& cov, double max_eigenvalue = 1e6) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);

  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = solver.eigenvectors();

  bool clamped = false;
  for (int i = 0; i < eigenvalues.size(); ++i) {
    if (eigenvalues(i) > max_eigenvalue) {
      eigenvalues(i) = max_eigenvalue;
      clamped = true;
    }
    if (eigenvalues(i) < 1e-12) {  // Enforce PSD
      eigenvalues(i) = 1e-12;
      clamped = true;
    }
  }

  if (clamped) {
    cov = eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
    LOG_WARN("Covariance clamped: max_eig=" << eigenvalues.maxCoeff());
  }
}

// In UKF update
Eigen::MatrixXd cov = ukf_.getCovariance();
clampCovariance(cov);
ukf_.setCovariance(cov);
```

---

## 5. Calibration Framework

### Factory Calibration

```cpp
struct FactoryCalibration {
  struct Imu {
    Eigen::Matrix3d scale;         // 3x3 scale factors
    Eigen::Matrix3d misalignment;  // Non-orthogonality
    Eigen::Vector3d accel_bias_0;  // At 25°C
    Eigen::Vector3d gyro_bias_0;
  } imu;

  struct Mag {
    Eigen::Vector3d hard_iron;     // Offset
    Eigen::Matrix3d soft_iron;     // Scale/rotation
    double median_norm;            // Expected |m|
  } mag;

  struct Camera {
    double fx, fy, cx, cy;         // Intrinsics
    Eigen::VectorXd distortion;    // k1,k2,p1,p2,k3 (radtan)
  } camera;

  struct Extrinsics {
    gtsam::Pose3 T_body_imu;
    gtsam::Pose3 T_body_camera;
    gtsam::Pose3 T_body_lidar;
  } extrinsics;

  std::string version = "1.0.0";
  std::string serial_number;
  uint64_t timestamp_utc;
  std::string checksum;  // SHA256 of serialized data

  // Serialize/deserialize
  void save(const std::string& filepath);
  static FactoryCalibration load(const std::string& filepath);
  bool verify() const;  // Check checksum
};
```

### Field Calibration

```cpp
struct FieldCalibration {
  struct MagRecal {
    Eigen::Vector3d hard_iron_delta;  // Update from factory
    double timestamp;
  } mag_recal;

  struct BaroOffset {
    double z_offset;  // "Tap to zero" altitude offset
    double timestamp;
  } baro_offset;

  struct OpticalFlowCheck {
    double scale_factor;  // Sanity check vs factory
    double focus_quality;
    bool passed;
  } of_check;

  std::string version = "1.0.0";
  std::string checksum;

  void save(const std::string& filepath);
  static FieldCalibration load(const std::string& filepath);
};
```

### Boot-Time Validation

```cpp
class CalibrationValidator {
 public:
  bool validate(const FactoryCalibration& factory,
                const FieldCalibration& field) {
    bool ok = true;

    // Verify checksums
    if (!factory.verify()) {
      LOG_ERROR("Factory calibration checksum failed!");
      ok = false;
    }

    if (!field.verify()) {
      LOG_ERROR("Field calibration checksum failed!");
      ok = false;
    }

    // Check versions
    if (factory.version != "1.0.0") {
      LOG_WARN("Factory calibration version mismatch: " << factory.version);
    }

    // Check age (field cal shouldn't be too old)
    uint64_t now = getCurrentTimeUTC();
    uint64_t field_age_days = (now - field.mag_recal.timestamp) / 86400;

    if (field_age_days > 30) {
      LOG_WARN("Field mag calibration is " << field_age_days
               << " days old. Consider recalibrating.");
    }

    // Sanity checks
    if (factory.camera.fx <= 0 || factory.camera.fy <= 0) {
      LOG_ERROR("Invalid camera intrinsics!");
      ok = false;
    }

    return ok;
  }
};
```

---

## 6. Thermal Compensation

### IMU Bias Polynomials

```cpp
struct ThermalModel {
  // Polynomial: bias(T) = c0 + c1*T + c2*T^2 + c3*T^3
  struct Polynomial {
    Eigen::Vector4d coeffs;  // [c0, c1, c2, c3]

    double evaluate(double temperature) const {
      double T = temperature;
      return coeffs(0) + coeffs(1)*T + coeffs(2)*T*T + coeffs(3)*T*T*T;
    }
  };

  Polynomial gyro_x, gyro_y, gyro_z;
  Polynomial accel_x, accel_y, accel_z;

  Eigen::Vector3d predictGyroBias(double temp) const {
    return Eigen::Vector3d(gyro_x.evaluate(temp),
                            gyro_y.evaluate(temp),
                            gyro_z.evaluate(temp));
  }

  Eigen::Vector3d predictAccelBias(double temp) const {
    return Eigen::Vector3d(accel_x.evaluate(temp),
                            accel_y.evaluate(temp),
                            accel_z.evaluate(temp));
  }

  void save(const std::string& filepath);
  static ThermalModel load(const std::string& filepath);
};

// In UKF propagate
Eigen::Vector3d gyro_thermal = thermal_model_.predictGyroBias(imu.temperature);
Eigen::Vector3d accel_thermal = thermal_model_.predictAccelBias(imu.temperature);

Eigen::Vector3d gyro_corrected = imu.gyro - gyro_bias_ - gyro_thermal;
Eigen::Vector3d accel_corrected = imu.accel - accel_bias_ - accel_thermal;
```

### Baro Temperature Correction

```cpp
struct BaroThermalModel {
  // Altitude correction: z_true = z_raw + c0 + c1*T + c2*(P - P0)
  double c0, c1, c2;  // Coefficients
  double P0 = 101325.0;  // Reference pressure (Pa)

  double correct(double z_raw, double temp_c, double pressure_pa) const {
    return z_raw + c0 + c1 * temp_c + c2 * (pressure_pa - P0);
  }
};
```

---

## 7. Resource Budgeting

### CPU Caps

```cpp
class ResourceMonitor {
 public:
  struct Budget {
    double cpu_percent_max = 25.0;  // % of single core
    size_t memory_mb_max = 300;
  };

  void setBudget(const Budget& budget) { budget_ = budget; }

  bool checkBudget() {
    // CPU usage (via /proc/stat or rusage)
    double cpu_percent = getCurrentCpuPercent();

    // Memory usage (via /proc/self/status or rusage)
    size_t memory_mb = getCurrentMemoryMB();

    bool within_budget = true;

    if (cpu_percent > budget_.cpu_percent_max) {
      LOG_WARN("CPU budget exceeded: " << cpu_percent
               << "% > " << budget_.cpu_percent_max << "%");
      within_budget = false;
      triggerLoadShedding();
    }

    if (memory_mb > budget_.memory_mb_max) {
      LOG_ERROR("Memory budget exceeded: " << memory_mb
                << "MB > " << budget_.memory_mb_max << "MB");
      within_budget = false;
    }

    return within_budget;
  }

 private:
  void triggerLoadShedding() {
    // Reduce smoother rate
    // Skip non-critical factors (e.g., mag if yaw stable)
    // Clear old telemetry buffers
  }

  Budget budget_;
};
```

---

## Telemetry Schema (Health Integration)

```json
{
  "t": 12.34,
  "sensor_health": {
    "lidar": {
      "score": 1.0,
      "is_stuck": false,
      "noise_level": 0.02,
      "reason": ""
    },
    "baro": {
      "score": 0.8,
      "noise_level": 0.3,
      "drift_rate": 0.01,
      "reason": "high_noise"
    },
    "mag": {
      "score": 1.0,
      "norm": 0.998,
      "interference": false,
      "reason": ""
    },
    "of": {
      "score": 0.95,
      "texture_quality": 0.92,
      "saturated": false,
      "reason": ""
    },
    "imu": {
      "score": 1.0,
      "temperature": 35.2,
      "gyro_saturated": false,
      "accel_saturated": false,
      "reason": ""
    }
  },
  "altitude_source": "lidar",
  "mode_transitions": 0,
  "watchdogs": {
    "ukf_propagate": {
      "last_ms": 0.12,
      "max_ms": 0.35,
      "violations": 0
    },
    "smoother_update": {
      "last_ms": 18.4,
      "max_ms": 25.1,
      "violations": 0
    }
  },
  "resource_usage": {
    "cpu_percent": 18.2,
    "memory_mb": 245
  }
}
```

---

## Implementation Checklist

### Week 1-3
- [ ] Monotonic clock implementation
- [ ] Per-sensor health monitors (all 5 sensors)
- [ ] Deterministic mode selection
- [ ] Deadline monitors for UKF/smoother
- [ ] NaN guards in critical paths

### Week 4-6
- [ ] Calibration persistence (factory + field)
- [ ] Thermal compensation (IMU + baro)
- [ ] Resource monitor with load-shedding

### Week 7-9
- [ ] Covariance clamping
- [ ] Health integration testing
- [ ] Telemetry schema update with health fields

---

## Acceptance Criteria

- [ ] Zero NaN/inf crashes in 10-hour soak test
- [ ] Clock skew handled gracefully (simulated ±100 ppm drift)
- [ ] Sensor failures detected within 1 second
- [ ] Mode transitions logged with reasons
- [ ] CPU/memory within budget on target HW
- [ ] Thermal compensation reduces bias drift by > 50%

---

**Next:** [INTEGRATION.md](./INTEGRATION.md) - ROS2/PX4/SDK integration
