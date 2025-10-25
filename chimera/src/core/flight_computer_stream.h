// CHIMERA Flight Computer - Streaming Mode API
// Wrapper for real-time ROS2/PX4 integration with incremental sensor processing
//
// This is an adapter over FlightComputer batch processing logic, refactored
// to support streaming sensor data as it arrives over ROS topics or MAVLink.

#pragma once

#include <deque>
#include <memory>
#include <optional>

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>

#include "core/ukf.h"
#include "core/smoother_wrapper.h"
#include "core/contract_monitor.h"
#include "core/sensor_health.h"
#include "core/flight_computer.h"

#include "utils/data_loaders.h"
#include "utils/telemetry.h"

namespace chimera {
namespace core {

/// Diagnostics data structure for external monitoring (ROS2, telemetry)
struct DiagnosticsData {
  // Sensor health scores (0.0 = failed, 1.0 = perfect)
  float lidar_health = 1.0f;
  float baro_health = 1.0f;
  float mag_health = 1.0f;
  float imu_health = 1.0f;
  float of_health = 1.0f;

  // System status
  std::string alt_source = "lidar";  // "lidar", "baro", "fusion"
  bool baro_takeover = false;
  bool watchdog_ok = true;
  int mode_transitions = 0;

  // Performance metrics
  double horizontal_drift_m_per_min = 0.0;
  double vertical_drift_m_per_min = 0.0;
  double gyro_bias_deg_per_s = 0.0;
  double accel_bias_m_per_s2 = 0.0;

  // Processing stats
  double ukf_cpu_ms = 0.0;
  double smoother_cpu_ms = 0.0;
  int imu_count = 0;
  int lidar_count = 0;
  int baro_count = 0;
  int mag_count = 0;
  int of_count = 0;

  // Graph stats
  double graph_total_error = 0.0;
  int graph_size = 0;
};

/// Streaming Flight Computer for real-time sensor processing
///
/// This class provides an incremental API suitable for ROS2/PX4 integration:
/// - Initialize with boot IMU buffer
/// - Process sensors as they arrive (imu, lidar, baro, mag, of)
/// - Query state for publishing (pose, velocity, covariance)
/// - Query diagnostics for monitoring
///
/// Design: Wraps/extracts logic from batch FlightComputer for streaming use
class FlightComputerStream {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor
  /// @param cfg Estimator configuration (UKF noise, thresholds, etc.)
  /// @param telemetry_path Optional path for JSONL telemetry logging
  explicit FlightComputerStream(
    const EstimatorConfig& cfg,
    const std::string& telemetry_path = "");

  /// Initialize estimator with boot IMU buffer
  /// @param boot_imu_buffer IMU samples collected during boot (100-400 samples)
  /// @return true if initialization successful
  ///
  /// This performs:
  /// - Auto-detection of IMU units (g vs m/s²)
  /// - Orientation boot (quaternion convention detection)
  /// - Gravity alignment validation
  /// - UKF initialization
  bool initialize(const std::vector<utils::ImuSample>& boot_imu_buffer);

  /// Process IMU sample (400 Hz)
  /// @param imu IMU measurement with gyro, accel, quat, temperature
  ///
  /// This triggers:
  /// - UKF propagation
  /// - Preintegration for factor graph
  /// - Smoother update (at configured rate, e.g., 5 Hz)
  /// - Reanchoring (periodic, e.g., every 3s)
  void processImu(const utils::ImuSample& imu);

  /// Process LiDAR range measurement
  /// @param lidar LiDAR sample with range_mean, range_min, range_max
  void processLidar(const utils::LidarSample& lidar);

  /// Process barometer measurement
  /// @param baro Barometer sample with altitude (MSL)
  void processBaro(const utils::BaroSample& baro);

  /// Process magnetometer measurement
  /// @param mag Magnetometer sample with magnetic field vector
  void processMag(const utils::MagSample& mag);

  /// Process optical flow measurement
  /// @param of Optical flow sample with rays
  void processOpticalFlow(const utils::OpticalFlowSample& of);

  /// Check if estimator is initialized
  bool isInitialized() const { return initialized_; }

  /// Get current pose estimate (NED frame)
  gtsam::Pose3 getPose() const;

  /// Get current velocity estimate (NED frame, m/s)
  Eigen::Vector3d getVelocity() const;

  /// Get current state covariance (15x15 error-state)
  /// Order: [pos, vel, rot, gyro_bias, accel_bias]
  Eigen::Matrix<double, 15, 15> getCovariance() const;

  /// Get current gyro bias estimate (rad/s)
  Eigen::Vector3d getGyroBias() const;

  /// Get current accel bias estimate (m/s²)
  Eigen::Vector3d getAccelBias() const;

  /// Get current timestamp (seconds)
  double getTimestamp() const { return last_imu_t_.value_or(0.0); }

  /// Get diagnostics data for monitoring
  DiagnosticsData getDiagnostics() const;

  /// Get altitude above ground level (meters, positive = up)
  double getAGL() const { return ukf_ ? -ukf_->getPose().z() : 0.0; }

  /// Emit telemetry to JSONL file (for logging/debugging)
  void emitTelemetry();

private:
  // Configuration
  EstimatorConfig cfg_;
  std::string telemetry_path_;

  // Core estimators
  std::unique_ptr<UKF> ukf_;
  std::unique_ptr<SmootherWrapper> smoother_;
  std::unique_ptr<ContractMonitor> monitor_;
  std::unique_ptr<utils::Telemetry> telemetry_;

  // Sensor health monitors
  std::unique_ptr<LidarHealthMonitor> lidar_health_;
  std::unique_ptr<BaroHealthMonitor> baro_health_;
  std::unique_ptr<MagHealthMonitor> mag_health_;
  std::unique_ptr<ImuHealthMonitor> imu_health_;
  std::unique_ptr<OpticalFlowHealthMonitor> of_health_;

  // Preintegration
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> pim_;

  // State
  bool initialized_ = false;
  std::optional<double> last_imu_t_;
  std::optional<double> last_smoother_t_;
  std::optional<double> last_reanchor_t_;
  int key_ = 0;

  // Boot detection state
  gtsam::Rot3 init_Rwb_;
  double gyro_scale_ = 1.0;
  double baro_baseline_msl_ = 0.0;
  double init_agl_m_ = 0.0;

  // Sensor buffers for matching/interpolation
  std::deque<utils::LidarSample> lidar_buffer_;
  std::deque<utils::BaroSample> baro_buffer_;
  std::deque<utils::MagSample> mag_buffer_;

  // Counters
  int lidar_updates_ = 0;
  int baro_updates_ = 0;
  int mag_updates_ = 0;
  int of_updates_ = 0;
  int mode_transitions_ = 0;

  // Cached health scores
  float last_lidar_health_ = 1.0f;
  float last_baro_health_ = 1.0f;
  float last_mag_health_ = 1.0f;
  float last_imu_health_ = 1.0f;
  float last_of_health_ = 1.0f;

  // Diagnostics
  double last_graph_error_ = 0.0;
  int last_graph_size_ = 0;

  // Internal methods (extracted from batch FlightComputer)
  void runSmootherUpdate();
  void reanchorUKF();
  bool shouldTriggerBaroTakeover() const;
  void handleBaroTakeover();
};

}  // namespace core
}  // namespace chimera
