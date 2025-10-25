// CHIMERA Watchdog Infrastructure
// Critical safety monitors for runtime fault detection
// Implements specifications from docs/commercialization/ROADMAP.md Phase 1 Week 2

#pragma once

#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

namespace chimera {
namespace core {

// =================== NaN/Inf Guard ===================
// Validates numerical stability of state vectors and matrices
class NaNGuard {
 public:
  struct Result {
    bool is_valid = true;
    std::string reason;

    operator bool() const { return is_valid; }
  };

  // Check scalar value
  static Result check(double value, const std::string& name) {
    Result r;
    if (std::isnan(value)) {
      r.is_valid = false;
      r.reason = name + " is NaN";
    } else if (std::isinf(value)) {
      r.is_valid = false;
      r.reason = name + " is Inf";
    }
    return r;
  }

  // Check Eigen vector
  template<typename Derived>
  static Result check(const Eigen::MatrixBase<Derived>& vec, const std::string& name) {
    Result r;
    if (!vec.allFinite()) {
      r.is_valid = false;
      r.reason = name + " contains NaN/Inf";

      // Detailed diagnostics
      for (int i = 0; i < vec.size(); ++i) {
        if (std::isnan(vec(i))) {
          r.reason += " [" + std::to_string(i) + "]=NaN";
        } else if (std::isinf(vec(i))) {
          r.reason += " [" + std::to_string(i) + "]=Inf";
        }
      }
    }
    return r;
  }

  // Check Eigen matrix
  template<typename Derived>
  static Result checkMatrix(const Eigen::MatrixBase<Derived>& mat, const std::string& name) {
    Result r;
    if (!mat.allFinite()) {
      r.is_valid = false;
      r.reason = name + " contains NaN/Inf";

      // Detailed diagnostics
      for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
          if (std::isnan(mat(i, j))) {
            r.reason += " [" + std::to_string(i) + "," + std::to_string(j) + "]=NaN";
          } else if (std::isinf(mat(i, j))) {
            r.reason += " [" + std::to_string(i) + "," + std::to_string(j) + "]=Inf";
          }
        }
      }
    }
    return r;
  }

  // Check GTSAM Pose3
  static Result check(const gtsam::Pose3& pose, const std::string& name) {
    Result r;

    // Check translation
    auto t_check = check(pose.translation(), name + ".translation");
    if (!t_check) return t_check;

    // Check rotation matrix
    Eigen::Matrix3d R = pose.rotation().matrix();
    auto R_check = checkMatrix(R, name + ".rotation");
    if (!R_check) return R_check;

    return r;
  }

  // Check GTSAM ImuBias
  static Result check(const gtsam::imuBias::ConstantBias& bias, const std::string& name) {
    Result r;

    auto acc_check = check(bias.accelerometer(), name + ".accel");
    if (!acc_check) return acc_check;

    auto gyro_check = check(bias.gyroscope(), name + ".gyro");
    if (!gyro_check) return gyro_check;

    return r;
  }
};

// =================== Deadline Monitor ===================
// Tracks sensor update deadlines to detect timeouts
class DeadlineMonitor {
 public:
  struct SensorDeadline {
    std::string name;
    double last_update_time = 0.0;
    double timeout_threshold_s = 1.0;  // Default 1 second timeout
    bool ever_updated = false;
  };

  struct ViolationReport {
    bool has_violations = false;
    std::vector<std::string> timed_out_sensors;
    std::vector<double> time_since_update;
  };

  DeadlineMonitor() = default;

  // Register a sensor with timeout threshold
  void registerSensor(const std::string& name, double timeout_s) {
    SensorDeadline deadline;
    deadline.name = name;
    deadline.timeout_threshold_s = timeout_s;
    sensors_[name] = deadline;
  }

  // Update sensor timestamp
  void update(const std::string& name, double current_time) {
    auto it = sensors_.find(name);
    if (it != sensors_.end()) {
      it->second.last_update_time = current_time;
      it->second.ever_updated = true;
    }
  }

  // Check for deadline violations
  ViolationReport check(double current_time) const {
    ViolationReport report;

    for (const auto& kv : sensors_) {
      const auto& deadline = kv.second;

      // Skip sensors that have never been updated (not yet available)
      if (!deadline.ever_updated) continue;

      double time_since = current_time - deadline.last_update_time;

      if (time_since > deadline.timeout_threshold_s) {
        report.has_violations = true;
        report.timed_out_sensors.push_back(deadline.name);
        report.time_since_update.push_back(time_since);
      }
    }

    return report;
  }

  // Reset all deadlines (e.g., after system reset)
  void reset() {
    for (auto& kv : sensors_) {
      kv.second.last_update_time = 0.0;
      kv.second.ever_updated = false;
    }
  }

  // Get sensor info for debugging
  const std::map<std::string, SensorDeadline>& sensors() const { return sensors_; }

 private:
  std::map<std::string, SensorDeadline> sensors_;
};

// =================== Covariance Health Monitor ===================
// Monitors covariance matrix health (PSD, conditioning, divergence)
class CovarianceHealthMonitor {
 public:
  struct Health {
    bool is_healthy = true;
    std::string reason;

    // Diagnostics
    double min_eigenvalue = 0.0;
    double max_eigenvalue = 0.0;
    double condition_number = 1.0;
    bool is_psd = true;              // Positive Semi-Definite
    bool is_well_conditioned = true;
    bool is_diverging = false;

    operator bool() const { return is_healthy; }
  };

  struct Config {
    double min_eigenvalue_thresh = 1e-12;  // Minimum eigenvalue for PSD
    double condition_number_thresh = 1e8;  // Max condition number
    double divergence_thresh = 1e6;        // Max diagonal element (variance)
  };

  CovarianceHealthMonitor() : cfg_() {}
  explicit CovarianceHealthMonitor(const Config& cfg) : cfg_(cfg) {}

  // Evaluate covariance matrix health
  Health evaluate(const Eigen::MatrixXd& cov, const std::string& name = "Covariance") {
    Health h;
    h.is_healthy = true;

    // 1. Check for NaN/Inf
    auto nan_check = NaNGuard::checkMatrix(cov, name);
    if (!nan_check) {
      h.is_healthy = false;
      h.reason = nan_check.reason;
      return h;
    }

    // 2. Check symmetry
    if (!cov.isApprox(cov.transpose(), 1e-6)) {
      h.is_healthy = false;
      h.reason = name + " is not symmetric";
      return h;
    }

    // 3. Eigenvalue analysis (PSD check + conditioning)
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
    if (solver.info() != Eigen::Success) {
      h.is_healthy = false;
      h.reason = name + " eigenvalue decomposition failed";
      return h;
    }

    Eigen::VectorXd eigenvalues = solver.eigenvalues();
    h.min_eigenvalue = eigenvalues.minCoeff();
    h.max_eigenvalue = eigenvalues.maxCoeff();

    // Check PSD (all eigenvalues >= 0)
    if (h.min_eigenvalue < -cfg_.min_eigenvalue_thresh) {
      h.is_psd = false;
      h.is_healthy = false;
      h.reason = name + " not PSD (min_eig=" + std::to_string(h.min_eigenvalue) + ")";
    }

    // Check conditioning (ratio of max/min eigenvalue)
    if (h.min_eigenvalue > cfg_.min_eigenvalue_thresh) {
      h.condition_number = h.max_eigenvalue / h.min_eigenvalue;
      if (h.condition_number > cfg_.condition_number_thresh) {
        h.is_well_conditioned = false;
        h.is_healthy = false;
        h.reason = name + " ill-conditioned (cond=" + std::to_string(h.condition_number) + ")";
      }
    } else {
      h.condition_number = std::numeric_limits<double>::infinity();
      h.is_well_conditioned = false;
    }

    // 4. Check for divergence (diagonal elements)
    for (int i = 0; i < cov.rows(); ++i) {
      if (cov(i, i) > cfg_.divergence_thresh) {
        h.is_diverging = true;
        h.is_healthy = false;
        h.reason = name + " diverging (var[" + std::to_string(i) + "]=" +
                   std::to_string(cov(i, i)) + ")";
        break;
      }
    }

    return h;
  }

  // Quick PSD check without full diagnostics
  static bool isPSD(const Eigen::MatrixXd& cov) {
    Eigen::LDLT<Eigen::MatrixXd> ldlt(cov);
    return ldlt.isPositive();
  }

 private:
  Config cfg_;
};

// =================== Time Synchronization Guards ===================
// Detects out-of-order sensor data and monotonicity violations

class MonotonicClock {
 public:
  MonotonicClock() = default;

  struct Result {
    bool is_valid = true;
    std::string reason;
    double time_jump = 0.0;

    operator bool() const { return is_valid; }
  };

  // Check if timestamp is monotonically increasing
  Result check(double timestamp, const std::string& sensor_name) {
    Result r;

    auto it = last_times_.find(sensor_name);
    if (it != last_times_.end()) {
      double last_time = it->second;

      if (timestamp < last_time) {
        r.is_valid = false;
        r.time_jump = timestamp - last_time;
        r.reason = sensor_name + " time went backwards: " +
                   std::to_string(last_time) + " -> " + std::to_string(timestamp) +
                   " (jump=" + std::to_string(r.time_jump) + "s)";
      }
    }

    last_times_[sensor_name] = timestamp;
    return r;
  }

  // Reset all tracked times
  void reset() {
    last_times_.clear();
  }

 private:
  std::map<std::string, double> last_times_;
};

class OutOfOrderDetector {
 public:
  struct Config {
    double tolerance_s = 0.01;  // Allow 10ms out-of-order (sensor jitter)
  };

  OutOfOrderDetector() : cfg_() {}
  explicit OutOfOrderDetector(const Config& cfg) : cfg_(cfg) {}

  struct Result {
    bool is_in_order = true;
    std::string reason;
    double time_delta = 0.0;

    operator bool() const { return is_in_order; }
  };

  // Check if sensor timestamp is in reasonable order with system time
  Result check(double sensor_time, double system_time, const std::string& sensor_name) {
    Result r;

    double delta = sensor_time - system_time;
    r.time_delta = delta;

    // Sensor timestamp should not be significantly ahead of system time
    if (delta > cfg_.tolerance_s) {
      r.is_in_order = false;
      r.reason = sensor_name + " timestamp ahead of system time by " +
                 std::to_string(delta) + "s";
    }

    // Sensor timestamp should not be too far behind (check for stale data)
    // This is handled by DeadlineMonitor, so we only check forward jumps here

    return r;
  }

 private:
  Config cfg_;
};

// =================== Aggregate Watchdog System ===================
// Combines all watchdogs for unified health checking

class WatchdogSystem {
 public:
  struct Report {
    bool all_healthy = true;
    std::vector<std::string> failures;

    DeadlineMonitor::ViolationReport deadline_violations;

    operator bool() const { return all_healthy; }
  };

  WatchdogSystem() = default;

  // Accessors for individual watchdogs
  DeadlineMonitor& deadlineMonitor() { return deadline_mon_; }
  CovarianceHealthMonitor& covarianceMonitor() { return cov_mon_; }
  MonotonicClock& monotonicClock() { return monotonic_clock_; }
  OutOfOrderDetector& outOfOrderDetector() { return ood_detector_; }

  // Check NaN guard
  void checkNaN(const NaNGuard::Result& result) {
    if (!result.is_valid) {
      last_report_.all_healthy = false;
      last_report_.failures.push_back("NaN: " + result.reason);
    }
  }

  // Check deadline violations
  void checkDeadlines(double current_time) {
    auto violations = deadline_mon_.check(current_time);
    if (violations.has_violations) {
      last_report_.all_healthy = false;
      last_report_.deadline_violations = violations;
      for (const auto& sensor : violations.timed_out_sensors) {
        last_report_.failures.push_back("Deadline: " + sensor + " timeout");
      }
    }
  }

  // Check covariance health
  void checkCovariance(const CovarianceHealthMonitor::Health& health) {
    if (!health.is_healthy) {
      last_report_.all_healthy = false;
      last_report_.failures.push_back("Covariance: " + health.reason);
    }
  }

  // Check monotonic time
  void checkMonotonic(const MonotonicClock::Result& result) {
    if (!result.is_valid) {
      last_report_.all_healthy = false;
      last_report_.failures.push_back("Monotonic: " + result.reason);
    }
  }

  // Get comprehensive report
  Report getReport() const { return last_report_; }

  // Clear report (call at start of each cycle)
  void clearReport() {
    last_report_ = Report();
  }

  // Reset all watchdogs
  void reset() {
    deadline_mon_.reset();
    monotonic_clock_.reset();
    last_report_ = Report();
  }

 private:
  DeadlineMonitor deadline_mon_;
  CovarianceHealthMonitor cov_mon_;
  MonotonicClock monotonic_clock_;
  OutOfOrderDetector ood_detector_;
  Report last_report_;
};

}  // namespace core
}  // namespace chimera
