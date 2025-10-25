// CHIMERA Flight Computer - Streaming Mode Implementation
// Real-time sensor processing for ROS2/PX4 integration

#include "core/flight_computer_stream.h"
#include "core/flight_computer.h"
#include "utils/sensor_helpers.h"

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>
#include <cmath>

namespace chimera {
namespace core {

using utils::ImuSample;
using utils::LidarSample;
using utils::BaroSample;
using utils::MagSample;
using utils::OpticalFlowSample;

FlightComputerStream::FlightComputerStream(
    const EstimatorConfig& cfg,
    const std::string& telemetry_path)
    : cfg_(cfg), telemetry_path_(telemetry_path) {

  // Initialize UKF
  ukf_ = std::make_unique<UKF>(cfg_.ukf_noise, cfg_.g);

  // Initialize smoother
  smoother_ = std::make_unique<SmootherWrapper>(cfg_.lag_seconds);

  // Initialize contract monitor
  monitor_ = std::make_unique<ContractMonitor>(cfg_.max_gap_s);

  // Initialize telemetry (if path provided)
  if (!telemetry_path_.empty()) {
    telemetry_ = std::make_unique<utils::Telemetry>(telemetry_path_);
  }

  // Initialize sensor health monitors
  lidar_health_ = std::make_unique<LidarHealthMonitor>();
  baro_health_ = std::make_unique<BaroHealthMonitor>();
  mag_health_ = std::make_unique<MagHealthMonitor>();
  imu_health_ = std::make_unique<ImuHealthMonitor>();
  of_health_ = std::make_unique<OpticalFlowHealthMonitor>();

  std::cout << "[FlightComputerStream] Initialized (streaming mode)\n";
}

bool FlightComputerStream::initialize(const std::vector<ImuSample>& boot_imu_buffer) {
  if (boot_imu_buffer.size() < 50) {
    std::cerr << "[FlightComputerStream] Insufficient IMU samples for boot: "
              << boot_imu_buffer.size() << " (need ≥50)\n";
    return false;
  }

  std::cout << "[FlightComputerStream] Initializing with "
            << boot_imu_buffer.size() << " IMU samples\n";

  // Step 1: Auto-detect IMU units (g vs m/s²)
  // Compute component-wise median
  std::vector<double> ax_vals, ay_vals, az_vals;
  for (const auto& imu : boot_imu_buffer) {
    ax_vals.push_back(imu.accel.x());
    ay_vals.push_back(imu.accel.y());
    az_vals.push_back(imu.accel.z());
  }

  Eigen::Vector3d accel_median;
  accel_median.x() = utils::medianScalar(ax_vals);
  accel_median.y() = utils::medianScalar(ay_vals);
  accel_median.z() = utils::medianScalar(az_vals);

  double accel_norm = accel_median.norm();

  if (accel_norm > 1.0 && accel_norm < 12.0) {
    // m/s² units
    std::cout << "[IMU BOOT] accel_norm=" << accel_norm << " m/s² → m/s² units\n";
  } else if (accel_norm > 0.8 && accel_norm < 1.2) {
    // g units, need to scale
    std::cout << "[IMU BOOT] accel_norm=" << accel_norm << " g → converting to m/s²\n";
    accel_median *= cfg_.g;
    // Note: In real implementation, would scale all subsequent IMU measurements
  } else {
    std::cerr << "[IMU BOOT] Unexpected accel norm: " << accel_norm << "\n";
  }

  // Step 2: Orientation boot - detect quaternion convention and frame
  Eigen::Vector4d quat_sample = boot_imu_buffer[boot_imu_buffer.size()/2].quat;
  auto orient_boot = utils::pickRwb_from_quat(quat_sample, accel_median);
  init_Rwb_ = orient_boot.Rwb;

  std::cout << "[ORIENT BOOT] picked " << orient_boot.tag
            << " g_err=" << orient_boot.g_err << " m/s²\n";

  // Step 3: Gravity alignment validation
  Eigen::Vector3d g_NED(0, 0, cfg_.g);
  Eigen::Vector3d residual = init_Rwb_.matrix() * accel_median + g_NED;
  double residual_norm = residual.norm();

  if (residual_norm < 0.8) {
    std::cout << "[BOOT OK] gravity residual = " << residual_norm << " m/s²\n";
  } else {
    std::cout << "[BOOT WARN] gravity residual = " << residual_norm
              << " m/s² (expected <0.8)\n";
  }

  // Step 4: Initialize UKF
  gtsam::Pose3 init_pose(init_Rwb_, gtsam::Point3(0, 0, 0));
  Eigen::Vector3d init_velocity = Eigen::Vector3d::Zero();

  Eigen::Matrix<double, 15, 15> init_cov = Eigen::Matrix<double, 15, 15>::Identity();
  init_cov.block<3,3>(0,0) *= 0.01;   // position: 10cm std
  init_cov.block<3,3>(3,3) *= 0.1;    // velocity: 10cm/s std
  init_cov.block<3,3>(6,6) *= 0.01;   // rotation: ~0.1 rad std
  init_cov.block<3,3>(9,9) *= 0.001;  // gyro bias: 0.001 rad/s std
  init_cov.block<3,3>(12,12) *= 0.01; // accel bias: 0.01 m/s² std

  ukf_->initialize(init_pose, init_velocity, init_cov);

  // Step 5: Initialize preintegration params
  auto imu_params = gtsam::PreintegrationParams::MakeSharedD(cfg_.g);
  imu_params->setAccelerometerCovariance(
    cfg_.ukf_noise.accel_noise_density * cfg_.ukf_noise.accel_noise_density * Eigen::Matrix3d::Identity());
  imu_params->setGyroscopeCovariance(
    cfg_.ukf_noise.gyro_noise_density * cfg_.ukf_noise.gyro_noise_density * Eigen::Matrix3d::Identity());
  imu_params->setIntegrationCovariance(1e-7 * Eigen::Matrix3d::Identity());
  imu_params->setUse2ndOrderCoriolis(false);

  gtsam::imuBias::ConstantBias zero_bias;
  pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(imu_params, zero_bias);

  // Set initial timestamp from first IMU sample
  last_imu_t_ = boot_imu_buffer.back().t;
  last_smoother_t_ = boot_imu_buffer.back().t;
  last_reanchor_t_ = boot_imu_buffer.back().t;

  initialized_ = true;
  std::cout << "[FlightComputerStream] Initialization complete!\n";

  return true;
}

void FlightComputerStream::processImu(const ImuSample& imu) {
  if (!initialized_) {
    std::cerr << "[FlightComputerStream] processImu() called before initialization!\n";
    return;
  }

  // Update timestamp
  double dt = last_imu_t_ ? (imu.t - *last_imu_t_) : 0.0;
  if (dt <= 0.0 || dt > 0.1) {
    last_imu_t_ = imu.t;
    return;  // Skip bad timestamp
  }

  // Step 1: UKF propagation
  UKF::ImuMeasurement ukf_imu;
  ukf_imu.timestamp = imu.t;
  ukf_imu.accel = imu.accel;
  ukf_imu.gyro = imu.gyro;
  ukf_imu.temperature = 25.0;  // TODO: Extract from IMU if available

  ukf_->propagate(ukf_imu);

  // Step 2: Preintegration for factor graph
  pim_->integrateMeasurement(imu.accel, imu.gyro, dt);

  // Step 3: Check if we should run smoother update
  double time_since_smoother = imu.t - last_smoother_t_.value_or(0.0);
  double smoother_period = 1.0 / cfg_.smoother_hz;

  if (time_since_smoother >= smoother_period) {
    runSmootherUpdate();
    last_smoother_t_ = imu.t;
  }

  // Step 4: Check if we should reanchor UKF
  double time_since_reanchor = imu.t - last_reanchor_t_.value_or(0.0);
  if (time_since_reanchor >= cfg_.reanchor_period_s) {
    reanchorUKF();
    last_reanchor_t_ = imu.t;
  }

  last_imu_t_ = imu.t;
}

void FlightComputerStream::processLidar(const LidarSample& lidar) {
  if (!initialized_) return;

  lidar_buffer_.push_back(lidar);
  if (lidar_buffer_.size() > 10) {
    lidar_buffer_.pop_front();
  }

  lidar_updates_++;

  // Evaluate health
  if (lidar_health_) {
    auto health = lidar_health_->evaluate(lidar);
    last_lidar_health_ = health.score;
  }
}

void FlightComputerStream::processBaro(const BaroSample& baro) {
  if (!initialized_) return;

  baro_buffer_.push_back(baro);
  if (baro_buffer_.size() > 10) {
    baro_buffer_.pop_front();
  }

  baro_updates_++;

  // Evaluate health
  if (baro_health_) {
    auto health = baro_health_->evaluate(baro);
    last_baro_health_ = health.score;
  }
}

void FlightComputerStream::processMag(const MagSample& mag) {
  if (!initialized_) return;

  mag_buffer_.push_back(mag);
  if (mag_buffer_.size() > 10) {
    mag_buffer_.pop_front();
  }

  mag_updates_++;

  // Evaluate health
  if (mag_health_) {
    auto health = mag_health_->evaluate(mag);
    last_mag_health_ = health.score;
  }
}

void FlightComputerStream::processOpticalFlow(const OpticalFlowSample& of) {
  if (!initialized_) return;

  of_updates_++;

  // Evaluate health
  if (of_health_) {
    auto health = of_health_->evaluate(of);
    last_of_health_ = health.score;
  }
}

gtsam::Pose3 FlightComputerStream::getPose() const {
  if (!initialized_) {
    return gtsam::Pose3::Identity();
  }
  return ukf_->getPose();
}

Eigen::Vector3d FlightComputerStream::getVelocity() const {
  if (!initialized_) {
    return Eigen::Vector3d::Zero();
  }
  return ukf_->getVelocity();
}

Eigen::Matrix<double, 15, 15> FlightComputerStream::getCovariance() const {
  if (!initialized_) {
    return Eigen::Matrix<double, 15, 15>::Identity();
  }
  return ukf_->getCovariance();
}

Eigen::Vector3d FlightComputerStream::getGyroBias() const {
  if (!initialized_) {
    return Eigen::Vector3d::Zero();
  }
  return ukf_->getGyroBias();
}

Eigen::Vector3d FlightComputerStream::getAccelBias() const {
  if (!initialized_) {
    return Eigen::Vector3d::Zero();
  }
  return ukf_->getAccelBias();
}

DiagnosticsData FlightComputerStream::getDiagnostics() const {
  DiagnosticsData diag;

  if (!initialized_) {
    return diag;  // Return defaults
  }

  // Sensor health scores (from cached evaluations)
  diag.lidar_health = last_lidar_health_;
  diag.baro_health = last_baro_health_;
  diag.mag_health = last_mag_health_;
  diag.imu_health = last_imu_health_;
  diag.of_health = last_of_health_;

  // Altitude source (simplified)
  if (diag.lidar_health > 0.7f) {
    diag.alt_source = "lidar";
  } else if (diag.baro_health > 0.5f) {
    diag.alt_source = "baro";
  } else {
    diag.alt_source = "degraded";
  }

  // Baro takeover status
  diag.baro_takeover = false;  // TODO: Track actual takeover state

  // Watchdog status
  diag.watchdog_ok = monitor_ ? monitor_->isContractOk(last_imu_t_.value_or(0.0)) : true;

  // Bias estimates
  Eigen::Vector3d gyro_bias = getGyroBias();
  Eigen::Vector3d accel_bias = getAccelBias();
  diag.gyro_bias_deg_per_s = gyro_bias.norm() * 57.2958;  // rad/s to deg/s
  diag.accel_bias_m_per_s2 = accel_bias.norm();

  // Counters
  diag.imu_count = last_imu_t_ ? static_cast<int>(*last_imu_t_ * 400) : 0;  // Approx
  diag.lidar_count = lidar_updates_;
  diag.baro_count = baro_updates_;
  diag.mag_count = mag_updates_;
  diag.of_count = of_updates_;

  // Graph stats
  diag.graph_total_error = last_graph_error_;
  diag.graph_size = last_graph_size_;

  // Mode transitions
  diag.mode_transitions = mode_transitions_;

  return diag;
}

void FlightComputerStream::emitTelemetry() {
  if (!telemetry_ || !initialized_) return;

  // Build telemetry JSON
  boost::json::object telem;
  telem["t"] = last_imu_t_.value_or(0.0);

  gtsam::Pose3 pose = getPose();
  Eigen::Vector3d vel = getVelocity();

  telem["agl_m"] = -pose.z();  // NED Z to AGL

  boost::json::array pos_arr;
  pos_arr.push_back(pose.x());
  pos_arr.push_back(pose.y());
  pos_arr.push_back(pose.z());
  telem["pos"] = pos_arr;

  boost::json::array vel_arr;
  vel_arr.push_back(vel.x());
  vel_arr.push_back(vel.y());
  vel_arr.push_back(vel.z());
  telem["vel"] = vel_arr;

  // Diagnostics
  auto diag = getDiagnostics();
  telem["lidar_health"] = diag.lidar_health;
  telem["baro_health"] = diag.baro_health;
  telem["mag_health"] = diag.mag_health;
  telem["of_health"] = diag.of_health;
  telem["alt_source"] = diag.alt_source;
  telem["gyro_bias_dps"] = diag.gyro_bias_deg_per_s;

  telemetry_->log(telem);
}

void FlightComputerStream::runSmootherUpdate() {
  // Simplified smoother update for streaming mode
  // TODO: Full implementation with factor graph construction

  // For now, just increment key
  key_++;

  // In full implementation:
  // 1. Create factor graph with IMU factors, range factors, etc.
  // 2. Add new values for X(key_), V(key_), B(key_)
  // 3. Call smoother_->update(graph, values, timestamps)
  // 4. Extract optimized state
  // 5. Reanchor UKF with optimized state

  // Placeholder: Track graph stats
  last_graph_size_ = key_;
  last_graph_error_ = 0.0;  // Would compute from smoother
}

void FlightComputerStream::reanchorUKF() {
  // Simplified reanchoring
  // In full implementation, extract optimized state from smoother and reset UKF

  // For now, just note that reanchoring occurred
  // std::cout << "[REANCHOR] t=" << last_imu_t_.value_or(0.0) << "\n";
}

bool FlightComputerStream::shouldTriggerBaroTakeover() const {
  // Check if altitude divergence exceeds threshold
  // Requires comparing UKF estimate with baro measurement
  return false;  // Placeholder
}

void FlightComputerStream::handleBaroTakeover() {
  // Baro takeover logic:
  // 1. Reduce lag window temporarily
  // 2. Reinitialize smoother with baro-anchored state
  // 3. Cut IMU preintegration chain
  // 4. Restore lag window after recovery

  mode_transitions_++;
}

}  // namespace core
}  // namespace chimera
