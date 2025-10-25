// Unit tests for CHIMERA Watchdog Infrastructure
// Tests NaN guards, deadline monitors, covariance health, and time synchronization

#include <gtest/gtest.h>
#include "core/watchdogs.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>

using namespace chimera::core;

// =================== NaN Guard Tests ===================
class NaNGuardTest : public ::testing::Test {};

TEST_F(NaNGuardTest, ValidScalar) {
  auto result = NaNGuard::check(42.0, "test_value");
  EXPECT_TRUE(result.is_valid);
  EXPECT_TRUE(result.reason.empty());
}

TEST_F(NaNGuardTest, DetectNaN) {
  double nan_val = std::nan("");
  auto result = NaNGuard::check(nan_val, "test_value");
  EXPECT_FALSE(result.is_valid);
  EXPECT_NE(result.reason.find("NaN"), std::string::npos);
}

TEST_F(NaNGuardTest, DetectInf) {
  double inf_val = std::numeric_limits<double>::infinity();
  auto result = NaNGuard::check(inf_val, "test_value");
  EXPECT_FALSE(result.is_valid);
  EXPECT_NE(result.reason.find("Inf"), std::string::npos);
}

TEST_F(NaNGuardTest, ValidVector) {
  Eigen::Vector3d vec(1.0, 2.0, 3.0);
  auto result = NaNGuard::check(vec, "test_vec");
  EXPECT_TRUE(result.is_valid);
}

TEST_F(NaNGuardTest, DetectNaNInVector) {
  Eigen::Vector3d vec(1.0, std::nan(""), 3.0);
  auto result = NaNGuard::check(vec, "test_vec");
  EXPECT_FALSE(result.is_valid);
  EXPECT_NE(result.reason.find("NaN"), std::string::npos);
}

TEST_F(NaNGuardTest, ValidMatrix) {
  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
  auto result = NaNGuard::checkMatrix(mat, "test_mat");
  EXPECT_TRUE(result.is_valid);
}

TEST_F(NaNGuardTest, DetectNaNInMatrix) {
  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
  mat(1, 1) = std::nan("");
  auto result = NaNGuard::checkMatrix(mat, "test_mat");
  EXPECT_FALSE(result.is_valid);
  EXPECT_NE(result.reason.find("NaN"), std::string::npos);
}

TEST_F(NaNGuardTest, ValidPose3) {
  gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(1, 2, 3));
  auto result = NaNGuard::check(pose, "test_pose");
  EXPECT_TRUE(result.is_valid);
}

TEST_F(NaNGuardTest, ValidImuBias) {
  gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.1, 0.2, 0.3), gtsam::Vector3(0.01, 0.02, 0.03));
  auto result = NaNGuard::check(bias, "test_bias");
  EXPECT_TRUE(result.is_valid);
}

// =================== Deadline Monitor Tests ===================
class DeadlineMonitorTest : public ::testing::Test {
 protected:
  DeadlineMonitor monitor_;

  void SetUp() override {
    monitor_.registerSensor("lidar", 0.2);  // 200ms timeout
    monitor_.registerSensor("imu", 0.05);   // 50ms timeout
  }
};

TEST_F(DeadlineMonitorTest, NoViolations_WithinDeadline) {
  monitor_.update("lidar", 0.0);
  monitor_.update("imu", 0.0);

  auto report = monitor_.check(0.04);  // 40ms later - both within deadline
  EXPECT_FALSE(report.has_violations);
  EXPECT_TRUE(report.timed_out_sensors.empty());
}

TEST_F(DeadlineMonitorTest, DetectTimeout_SingleSensor) {
  monitor_.update("lidar", 0.0);
  monitor_.update("imu", 0.0);

  auto report = monitor_.check(0.15);  // 150ms later - lidar OK (< 200ms), IMU timeout (> 50ms)
  EXPECT_TRUE(report.has_violations);
  EXPECT_EQ(report.timed_out_sensors.size(), 1);
  EXPECT_EQ(report.timed_out_sensors[0], "imu");
  EXPECT_NEAR(report.time_since_update[0], 0.15, 1e-6);
}

TEST_F(DeadlineMonitorTest, DetectTimeout_MultipleSensors) {
  monitor_.update("lidar", 0.0);
  monitor_.update("imu", 0.0);

  auto report = monitor_.check(0.5);  // 500ms later - both timeout
  EXPECT_TRUE(report.has_violations);
  EXPECT_EQ(report.timed_out_sensors.size(), 2);
}

TEST_F(DeadlineMonitorTest, IgnoreNeverUpdated) {
  // Don't update sensors, check immediately
  auto report = monitor_.check(1.0);
  EXPECT_FALSE(report.has_violations);  // Should ignore never-updated sensors
}

TEST_F(DeadlineMonitorTest, Reset_ClearsHistory) {
  monitor_.update("lidar", 0.0);
  monitor_.reset();

  // After reset, sensor should be treated as never updated
  auto report = monitor_.check(1.0);
  EXPECT_FALSE(report.has_violations);
}

// =================== Covariance Health Monitor Tests ===================
class CovarianceHealthMonitorTest : public ::testing::Test {
 protected:
  CovarianceHealthMonitor monitor_;
};

TEST_F(CovarianceHealthMonitorTest, HealthyCovariance_Identity) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  auto health = monitor_.evaluate(cov, "test_cov");

  EXPECT_TRUE(health.is_healthy);
  EXPECT_TRUE(health.is_psd);
  EXPECT_TRUE(health.is_well_conditioned);
  EXPECT_FALSE(health.is_diverging);
  EXPECT_NEAR(health.condition_number, 1.0, 1e-6);
}

TEST_F(CovarianceHealthMonitorTest, HealthyCovariance_Diagonal) {
  Eigen::Matrix3d cov = Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal();
  auto health = monitor_.evaluate(cov, "test_cov");

  EXPECT_TRUE(health.is_healthy);
  EXPECT_TRUE(health.is_psd);
  EXPECT_NEAR(health.min_eigenvalue, 0.1, 1e-6);
  EXPECT_NEAR(health.max_eigenvalue, 0.3, 1e-6);
  EXPECT_NEAR(health.condition_number, 3.0, 1e-6);
}

TEST_F(CovarianceHealthMonitorTest, DetectNegativeEigenvalue_NotPSD) {
  Eigen::Matrix3d cov;
  cov << 1, 0, 0,
         0, -0.1, 0,  // Negative eigenvalue
         0, 0, 1;

  auto health = monitor_.evaluate(cov, "test_cov");

  EXPECT_FALSE(health.is_healthy);
  EXPECT_FALSE(health.is_psd);
  EXPECT_LT(health.min_eigenvalue, 0.0);
}

TEST_F(CovarianceHealthMonitorTest, DetectIllConditioned) {
  CovarianceHealthMonitor::Config cfg;
  cfg.condition_number_thresh = 100.0;  // Strict threshold
  CovarianceHealthMonitor strict_monitor(cfg);

  Eigen::Matrix3d cov = Eigen::Vector3d(1e-6, 1.0, 1.0).asDiagonal();
  auto health = strict_monitor.evaluate(cov, "test_cov");

  EXPECT_FALSE(health.is_healthy);
  EXPECT_FALSE(health.is_well_conditioned);
  EXPECT_GT(health.condition_number, 100.0);
}

TEST_F(CovarianceHealthMonitorTest, DetectDivergence) {
  Eigen::Matrix3d cov = Eigen::Vector3d(1e7, 1.0, 1.0).asDiagonal();
  auto health = monitor_.evaluate(cov, "test_cov");

  EXPECT_FALSE(health.is_healthy);
  EXPECT_TRUE(health.is_diverging);
}

TEST_F(CovarianceHealthMonitorTest, DetectNonSymmetric) {
  Eigen::Matrix3d cov;
  cov << 1, 0.5, 0,
         0.3, 1, 0,  // Not symmetric (0.5 != 0.3)
         0, 0, 1;

  auto health = monitor_.evaluate(cov, "test_cov");

  EXPECT_FALSE(health.is_healthy);
  EXPECT_NE(health.reason.find("symmetric"), std::string::npos);
}

TEST_F(CovarianceHealthMonitorTest, DetectNaN) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  cov(1, 1) = std::nan("");

  auto health = monitor_.evaluate(cov, "test_cov");

  EXPECT_FALSE(health.is_healthy);
  EXPECT_NE(health.reason.find("NaN"), std::string::npos);
}

TEST_F(CovarianceHealthMonitorTest, QuickPSDCheck_Valid) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  EXPECT_TRUE(CovarianceHealthMonitor::isPSD(cov));
}

TEST_F(CovarianceHealthMonitorTest, QuickPSDCheck_Invalid) {
  Eigen::Matrix3d cov;
  cov << 1, 0, 0,
         0, -1, 0,
         0, 0, 1;
  EXPECT_FALSE(CovarianceHealthMonitor::isPSD(cov));
}

// =================== Monotonic Clock Tests ===================
class MonotonicClockTest : public ::testing::Test {
 protected:
  MonotonicClock clock_;
};

TEST_F(MonotonicClockTest, MonotonicIncreasing_Valid) {
  auto r1 = clock_.check(0.0, "sensor");
  EXPECT_TRUE(r1.is_valid);

  auto r2 = clock_.check(0.1, "sensor");
  EXPECT_TRUE(r2.is_valid);

  auto r3 = clock_.check(0.2, "sensor");
  EXPECT_TRUE(r3.is_valid);
}

TEST_F(MonotonicClockTest, DetectBackwardsTime) {
  clock_.check(1.0, "sensor");

  auto result = clock_.check(0.5, "sensor");  // Time went backwards
  EXPECT_FALSE(result.is_valid);
  EXPECT_NEAR(result.time_jump, -0.5, 1e-6);
  EXPECT_NE(result.reason.find("backwards"), std::string::npos);
}

TEST_F(MonotonicClockTest, MultipleSensors_Independent) {
  clock_.check(1.0, "sensor_a");
  clock_.check(2.0, "sensor_b");

  // sensor_b going backwards shouldn't affect sensor_a
  auto result_a = clock_.check(1.5, "sensor_a");
  EXPECT_TRUE(result_a.is_valid);

  auto result_b = clock_.check(1.5, "sensor_b");
  EXPECT_FALSE(result_b.is_valid);
}

TEST_F(MonotonicClockTest, Reset_ClearsHistory) {
  clock_.check(1.0, "sensor");
  clock_.reset();

  // After reset, can go back in time without violation
  auto result = clock_.check(0.5, "sensor");
  EXPECT_TRUE(result.is_valid);
}

// =================== Out of Order Detector Tests ===================
class OutOfOrderDetectorTest : public ::testing::Test {
 protected:
  OutOfOrderDetector detector_;
};

TEST_F(OutOfOrderDetectorTest, InOrder_Valid) {
  auto result = detector_.check(1.0, 1.0, "sensor");
  EXPECT_TRUE(result.is_in_order);
}

TEST_F(OutOfOrderDetectorTest, SlightlyAhead_WithinTolerance) {
  auto result = detector_.check(1.005, 1.0, "sensor");
  EXPECT_TRUE(result.is_in_order);  // Within 10ms tolerance
}

TEST_F(OutOfOrderDetectorTest, DetectSignificantlyAhead) {
  auto result = detector_.check(1.5, 1.0, "sensor");
  EXPECT_FALSE(result.is_in_order);
  EXPECT_NEAR(result.time_delta, 0.5, 1e-6);
}

TEST_F(OutOfOrderDetectorTest, Behind_Allowed) {
  // Sensor timestamp behind system time is OK (handled by DeadlineMonitor)
  auto result = detector_.check(0.5, 1.0, "sensor");
  EXPECT_TRUE(result.is_in_order);
}

// =================== Watchdog System Integration Tests ===================
class WatchdogSystemTest : public ::testing::Test {
 protected:
  WatchdogSystem watchdog_;

  void SetUp() override {
    watchdog_.deadlineMonitor().registerSensor("lidar", 0.2);
    watchdog_.deadlineMonitor().registerSensor("imu", 0.05);
  }
};

TEST_F(WatchdogSystemTest, AllHealthy_NoFailures) {
  watchdog_.clearReport();

  // Check valid NaN guard
  auto nan_result = NaNGuard::check(42.0, "test");
  watchdog_.checkNaN(nan_result);

  // Check valid covariance
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  auto cov_health = watchdog_.covarianceMonitor().evaluate(cov);
  watchdog_.checkCovariance(cov_health);

  auto report = watchdog_.getReport();
  EXPECT_TRUE(report.all_healthy);
  EXPECT_TRUE(report.failures.empty());
}

TEST_F(WatchdogSystemTest, NaNDetected_ReportFailure) {
  watchdog_.clearReport();

  auto nan_result = NaNGuard::check(std::nan(""), "test_value");
  watchdog_.checkNaN(nan_result);

  auto report = watchdog_.getReport();
  EXPECT_FALSE(report.all_healthy);
  EXPECT_EQ(report.failures.size(), 1);
  EXPECT_NE(report.failures[0].find("NaN"), std::string::npos);
}

TEST_F(WatchdogSystemTest, DeadlineViolation_ReportFailure) {
  watchdog_.clearReport();

  watchdog_.deadlineMonitor().update("lidar", 0.0);
  watchdog_.deadlineMonitor().update("imu", 0.0);

  watchdog_.checkDeadlines(0.3);  // IMU timeout

  auto report = watchdog_.getReport();
  EXPECT_FALSE(report.all_healthy);
  EXPECT_GT(report.failures.size(), 0);
  EXPECT_TRUE(report.deadline_violations.has_violations);
}

TEST_F(WatchdogSystemTest, MultipleFailures_Accumulated) {
  watchdog_.clearReport();

  // Add NaN failure
  auto nan_result = NaNGuard::check(std::nan(""), "value");
  watchdog_.checkNaN(nan_result);

  // Add covariance failure
  Eigen::Matrix3d bad_cov = Eigen::Matrix3d::Identity();
  bad_cov(1, 1) = -1.0;  // Not PSD
  auto cov_health = watchdog_.covarianceMonitor().evaluate(bad_cov);
  watchdog_.checkCovariance(cov_health);

  auto report = watchdog_.getReport();
  EXPECT_FALSE(report.all_healthy);
  EXPECT_GE(report.failures.size(), 2);
}

TEST_F(WatchdogSystemTest, ClearReport_ResetsState) {
  watchdog_.clearReport();

  // Add failure
  auto nan_result = NaNGuard::check(std::nan(""), "value");
  watchdog_.checkNaN(nan_result);

  EXPECT_FALSE(watchdog_.getReport().all_healthy);

  // Clear and verify reset
  watchdog_.clearReport();
  auto report = watchdog_.getReport();
  EXPECT_TRUE(report.all_healthy);
  EXPECT_TRUE(report.failures.empty());
}
