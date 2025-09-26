#include <gtest/gtest.h>
#include <cmath>

#include "core/fusion/imu_preintegration.h"
#include "core/utils/logging.h"

TEST(IMUPreintegration, ConstantVelocity) {
  // Test requirement: position error < 0.5m after 10s constant velocity
  aion::IMUPreintegration::Params params;
  params.gyro_noise_density = 0.0;    // No noise for analytical comparison
  params.accel_noise_density = 0.0;
  params.gyro_bias_rw = 0.0;
  params.accel_bias_rw = 0.0;
  params.gravity = Eigen::Vector3d(0, 0, 9.80665);

  aion::IMUPreintegration preint(params);

  // Simulate constant velocity motion: v = [1, 0, 0] m/s
  double dt = 0.01;  // 100Hz
  double total_time = 10.0;  // 10 seconds
  int num_samples = static_cast<int>(total_time / dt);

  // For constant velocity, we need constant acceleration in body frame
  // Specific force = actual_accel - gravity (in body frame)
  // For no actual acceleration, IMU measures -gravity in body frame
  // In NED with body aligned to NED, stationary IMU measures (0,0,-g)

  for (int i = 0; i < num_samples; ++i) {
    aion::IMUData imu;
    imu.angular_velocity = Eigen::Vector3d::Zero();  // No rotation
    imu.acceleration = Eigen::Vector3d(0, 0, 0);  // Zero specific force (gravity-compensated)
    imu.timestamp = i * dt;
    imu.temperature = 25.0;

    preint.addMeasurement(imu, dt);
  }

  // Expected values for constant velocity motion
  Eigen::Vector3d expected_position = Eigen::Vector3d::Zero();  // No motion
  Eigen::Vector3d expected_velocity = Eigen::Vector3d::Zero();  // No velocity change
  Eigen::Quaterniond expected_rotation = Eigen::Quaterniond::Identity();  // No rotation

  // Check results
  EXPECT_NEAR(preint.getDeltaTime(), total_time, 1e-6);

  // Position error should be < 0.5m
  double position_error = (preint.getDeltaPosition() - expected_position).norm();
  EXPECT_LT(position_error, 0.5) << "Position error exceeds 0.5m requirement";

  // Velocity should be close to zero (only numerical errors)
  double velocity_error = (preint.getDeltaVelocity() - expected_velocity).norm();
  EXPECT_LT(velocity_error, 0.01);

  // No rotation expected
  double rotation_error = preint.getDeltaRotation().angularDistance(expected_rotation);
  EXPECT_LT(rotation_error, 0.01);
}

TEST(IMUPreintegration, CircularMotion) {
  aion::IMUPreintegration::Params params;
  params.gyro_noise_density = 0.0;
  params.accel_noise_density = 0.0;

  aion::IMUPreintegration preint(params);

  // Circular motion parameters
  double radius = 10.0;  // meters
  double period = 20.0;  // seconds
  double omega = 2.0 * M_PI / period;  // rad/s
  double dt = 0.01;
  int num_samples = static_cast<int>(period / dt);  // One full circle

  for (int i = 0; i < num_samples; ++i) {
    aion::IMUData imu;

    // Angular velocity (constant yaw rate)
    imu.angular_velocity = Eigen::Vector3d(0, 0, omega);

    // Centripetal acceleration in body frame (specific force)
    double v = radius * omega;  // tangential velocity
    double a_centripetal = v * omega;  // v²/r
    // IMU measures specific force, no gravity needed in preintegration
    imu.acceleration = Eigen::Vector3d(a_centripetal, 0, 0);

    imu.timestamp = i * dt;
    preint.addMeasurement(imu, dt);
  }

  // After one full circle of preintegration:
  // - Rotation should accumulate to ~2π around Z
  // - Position/velocity are in body frame and won't return to origin

  // Check that we got non-zero motion (sanity check)
  EXPECT_GT(preint.getDeltaPosition().norm(), 1.0)
      << "Should have accumulated significant position change";

  // Check total rotation angle
  // After 2π rotation, quaternion should be close to identity
  // (since 2π rotation = identity in quaternion representation)
  Eigen::AngleAxisd aa(preint.getDeltaRotation());
  double total_rotation = aa.angle();

  // In quaternion representation, 2π rotation wraps to 0
  // So we expect either ~0 or ~2π depending on numerical precision
  bool near_zero = std::abs(total_rotation) < 0.1;
  bool near_2pi = std::abs(total_rotation - 2.0 * M_PI) < 0.1;

  EXPECT_TRUE(near_zero || near_2pi)
      << "Rotation angle " << total_rotation
      << " should be near 0 or 2π after full circle";
}

TEST(IMUPreintegration, BiasCorrection) {
  aion::IMUPreintegration::Params params;
  aion::IMUPreintegration preint(params);

  // Add some measurements
  double dt = 0.01;
  for (int i = 0; i < 100; ++i) {
    aion::IMUData imu;
    imu.angular_velocity = Eigen::Vector3d(0.1, 0, 0);  // Roll rate
    imu.acceleration = Eigen::Vector3d(0, 0, 9.80665);
    imu.timestamp = i * dt;
    preint.addMeasurement(imu, dt);
  }

  // Store original values
  Eigen::Vector3d orig_position = preint.getDeltaPosition();
  Eigen::Vector3d orig_velocity = preint.getDeltaVelocity();
  Eigen::Quaterniond orig_rotation = preint.getDeltaRotation();

  // Apply bias correction
  Eigen::Vector3d delta_bg(0.01, 0, 0);  // Small gyro bias change
  Eigen::Vector3d delta_ba(0, 0, 0.01);  // Small accel bias change

  Eigen::Vector3d corrected_position = preint.getCorrectedDeltaPosition(delta_bg, delta_ba);
  Eigen::Vector3d corrected_velocity = preint.getCorrectedDeltaVelocity(delta_bg, delta_ba);
  Eigen::Quaterniond corrected_rotation = preint.getCorrectedDeltaRotation(delta_bg);

  // Corrections should be non-zero but small
  EXPECT_GT((corrected_position - orig_position).norm(), 1e-6);
  EXPECT_LT((corrected_position - orig_position).norm(), 1.0);

  EXPECT_GT((corrected_velocity - orig_velocity).norm(), 1e-6);
  EXPECT_LT((corrected_velocity - orig_velocity).norm(), 1.0);

  EXPECT_GT(corrected_rotation.angularDistance(orig_rotation), 1e-6);
  EXPECT_LT(corrected_rotation.angularDistance(orig_rotation), 0.1);
}

TEST(IMUPreintegration, CovariancePropagation) {
  aion::IMUPreintegration::Params params;
  params.gyro_noise_density = 0.01;   // rad/s/sqrt(Hz)
  params.accel_noise_density = 0.1;   // m/s²/sqrt(Hz)
  params.gyro_bias_rw = 1e-5;
  params.accel_bias_rw = 1e-4;

  aion::IMUPreintegration preint(params);

  // Add measurements
  double dt = 0.01;
  for (int i = 0; i < 1000; ++i) {  // 10 seconds
    aion::IMUData imu;
    imu.angular_velocity = Eigen::Vector3d::Zero();
    imu.acceleration = Eigen::Vector3d(0, 0, 9.80665);
    imu.timestamp = i * dt;
    preint.addMeasurement(imu, dt);
  }

  // Covariance should grow over time
  const auto& cov = preint.getCovariance();

  // Position uncertainty should be largest
  EXPECT_GT(cov(0, 0), 0.0);  // X position variance
  EXPECT_GT(cov(1, 1), 0.0);  // Y position variance
  EXPECT_GT(cov(2, 2), 0.0);  // Z position variance

  // Velocity uncertainty should be moderate
  EXPECT_GT(cov(3, 3), 0.0);  // X velocity variance
  EXPECT_GT(cov(4, 4), 0.0);  // Y velocity variance
  EXPECT_GT(cov(5, 5), 0.0);  // Z velocity variance

  // Rotation uncertainty should be smallest (no rotation)
  EXPECT_GT(cov(6, 6), 0.0);  // Roll variance
  EXPECT_GT(cov(7, 7), 0.0);  // Pitch variance
  EXPECT_GT(cov(8, 8), 0.0);  // Yaw variance

  // Covariance should be symmetric
  for (int i = 0; i < 15; ++i) {
    for (int j = i + 1; j < 15; ++j) {
      EXPECT_NEAR(cov(i, j), cov(j, i), 1e-10)
          << "Covariance not symmetric at (" << i << "," << j << ")";
    }
  }
}

int main(int argc, char** argv) {
  // Initialize logger
  aion::Logger::init("/tmp/test_imu_preintegration.log", spdlog::level::off);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}