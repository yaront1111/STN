// Unit tests for CHIMERA Calibration Framework
// Tests factory defaults, field calibration, and sensor corrections

#include <gtest/gtest.h>
#include "utils/calibration.h"
#include <Eigen/Dense>

using namespace chimera::utils;

// =================== Factory Defaults Tests ===================
class CalibrationFactoryTest : public ::testing::Test {
 protected:
  CalibrationData factory_calib_;

  void SetUp() override {
    factory_calib_ = createFactoryDefaults();
  }
};

TEST_F(CalibrationFactoryTest, VersionIsSet) {
  EXPECT_EQ(factory_calib_.version, CALIB_VERSION);
  EXPECT_EQ(factory_calib_.version, "1.0.0");
}

TEST_F(CalibrationFactoryTest, TypeIsFactory) {
  EXPECT_EQ(factory_calib_.type, CalibrationType::FACTORY);
}

TEST_F(CalibrationFactoryTest, ImuDefaultsIdentity) {
  // Scale matrices should be identity
  EXPECT_TRUE(factory_calib_.imu.accel_scale.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(factory_calib_.imu.gyro_scale.isApprox(Eigen::Matrix3d::Identity()));

  // Bias should be zero
  EXPECT_TRUE(factory_calib_.imu.accel_bias.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(factory_calib_.imu.gyro_bias.isApprox(Eigen::Vector3d::Zero()));

  // Temperature polynomials should be zero
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(factory_calib_.imu.accel_temp_poly[i].isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(factory_calib_.imu.gyro_temp_poly[i].isApprox(Eigen::Vector3d::Zero()));
  }
}

TEST_F(CalibrationFactoryTest, MagDefaultsIdentity) {
  EXPECT_TRUE(factory_calib_.mag.hard_iron.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(factory_calib_.mag.soft_iron.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_NEAR(factory_calib_.mag.reference_norm, 1.0, 1e-6);
}

TEST_F(CalibrationFactoryTest, BaroDefaultsZero) {
  EXPECT_NEAR(factory_calib_.baro.pressure_offset, 0.0, 1e-6);
  EXPECT_NEAR(factory_calib_.baro.field_offset_agl, 0.0, 1e-6);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(factory_calib_.baro.temp_poly[i], 0.0, 1e-6);
  }
}

TEST_F(CalibrationFactoryTest, LidarDefaultsPassthrough) {
  EXPECT_NEAR(factory_calib_.lidar.range_offset, 0.0, 1e-6);
  EXPECT_NEAR(factory_calib_.lidar.range_scale, 1.0, 1e-6);
  EXPECT_NEAR(factory_calib_.lidar.pitch_angle_deg, 0.0, 1e-6);
}

TEST_F(CalibrationFactoryTest, CameraReasonableDefaults) {
  EXPECT_GT(factory_calib_.camera.fx, 0.0);
  EXPECT_GT(factory_calib_.camera.fy, 0.0);
  EXPECT_TRUE(factory_calib_.camera.T_body_camera.isApprox(Eigen::Matrix4d::Identity()));
}

// =================== Calibration Manager Tests ===================
class CalibrationManagerTest : public ::testing::Test {
 protected:
  CalibrationManager manager_;

  void SetUp() override {
    manager_.data() = createFactoryDefaults();
  }
};

TEST_F(CalibrationManagerTest, ApplyImuAccel_FactoryDefault) {
  // Factory default: identity scale, zero bias
  Eigen::Vector3d raw_accel(0.0, 0.0, 9.81);
  auto corrected = manager_.applyImuAccel(raw_accel);

  // Should be unchanged with factory defaults
  EXPECT_TRUE(corrected.isApprox(raw_accel));
}

TEST_F(CalibrationManagerTest, ApplyImuAccel_WithBias) {
  // Set non-zero bias
  manager_.data().imu.accel_bias = Eigen::Vector3d(0.1, 0.2, 0.3);

  Eigen::Vector3d raw_accel(1.0, 2.0, 3.0);
  auto corrected = manager_.applyImuAccel(raw_accel);

  // Should subtract bias
  Eigen::Vector3d expected = raw_accel - manager_.data().imu.accel_bias;
  EXPECT_TRUE(corrected.isApprox(expected));
}

TEST_F(CalibrationManagerTest, ApplyImuAccel_WithScale) {
  // Set non-identity scale
  manager_.data().imu.accel_scale = 2.0 * Eigen::Matrix3d::Identity();

  Eigen::Vector3d raw_accel(1.0, 2.0, 3.0);
  auto corrected = manager_.applyImuAccel(raw_accel);

  // Should apply scale
  Eigen::Vector3d expected = 2.0 * raw_accel;
  EXPECT_TRUE(corrected.isApprox(expected));
}

TEST_F(CalibrationManagerTest, ApplyMag_FactoryDefault) {
  Eigen::Vector3d raw_mag(1.0, 0.5, 0.3);
  auto corrected = manager_.applyMag(raw_mag);

  // Factory default: identity, so unchanged
  EXPECT_TRUE(corrected.isApprox(raw_mag));
}

TEST_F(CalibrationManagerTest, ApplyMag_WithHardIron) {
  manager_.data().mag.hard_iron = Eigen::Vector3d(0.1, 0.2, 0.3);

  Eigen::Vector3d raw_mag(1.0, 0.5, 0.3);
  auto corrected = manager_.applyMag(raw_mag);

  // Should subtract hard-iron
  Eigen::Vector3d expected = raw_mag - manager_.data().mag.hard_iron;
  EXPECT_TRUE(corrected.isApprox(expected));
}

TEST_F(CalibrationManagerTest, ApplyBaro_FactoryDefault) {
  double raw_altitude = 100.0;
  auto corrected = manager_.applyBaro(raw_altitude);

  // Factory default: zero offset
  EXPECT_NEAR(corrected, raw_altitude, 1e-6);
}

TEST_F(CalibrationManagerTest, ApplyBaro_WithFieldOffset) {
  manager_.data().baro.field_offset_agl = 5.0;  // 5m offset

  double raw_altitude = 100.0;
  auto corrected = manager_.applyBaro(raw_altitude);

  EXPECT_NEAR(corrected, 105.0, 1e-6);
}

TEST_F(CalibrationManagerTest, ApplyLidar_FactoryDefault) {
  double raw_range = 10.0;
  auto corrected = manager_.applyLidar(raw_range);

  // Factory default: scale=1, offset=0
  EXPECT_NEAR(corrected, raw_range, 1e-6);
}

TEST_F(CalibrationManagerTest, ApplyLidar_WithOffset) {
  manager_.data().lidar.range_offset = 0.1;  // 10cm offset

  double raw_range = 10.0;
  auto corrected = manager_.applyLidar(raw_range);

  EXPECT_NEAR(corrected, 10.1, 1e-6);
}

TEST_F(CalibrationManagerTest, ApplyLidar_WithScale) {
  manager_.data().lidar.range_scale = 0.001;  // mm to m conversion

  double raw_range = 10000.0;  // 10000mm
  auto corrected = manager_.applyLidar(raw_range);

  EXPECT_NEAR(corrected, 10.0, 1e-6);  // 10m
}

TEST_F(CalibrationManagerTest, ApplyLidar_WithPitchAngle) {
  manager_.data().lidar.pitch_angle_deg = 30.0;  // 30 degrees down

  double raw_range = 10.0;
  auto corrected = manager_.applyLidar(raw_range);

  // Should correct for angle: range * cos(30°) ≈ 8.66
  EXPECT_NEAR(corrected, 10.0 * std::cos(30.0 * M_PI / 180.0), 1e-3);
}

// =================== Field Calibration Tests ===================
TEST_F(CalibrationManagerTest, UpdateBaroFieldOffset) {
  double initial_offset = manager_.data().baro.field_offset_agl;

  manager_.updateBaroFieldOffset(2.5);

  EXPECT_NEAR(manager_.data().baro.field_offset_agl, 2.5, 1e-6);
  EXPECT_NE(manager_.data().baro.field_calib_time, 0);
}

TEST_F(CalibrationManagerTest, UpdateMagFieldCalibration) {
  // Generate sample data with known hard-iron offset
  std::vector<Eigen::Vector3d> samples;
  Eigen::Vector3d true_offset(0.1, 0.2, 0.3);

  for (int i = 0; i < 100; ++i) {
    double angle = 2.0 * M_PI * i / 100.0;
    Eigen::Vector3d sample(std::cos(angle), std::sin(angle), 0.0);
    sample += true_offset;
    samples.push_back(sample);
  }

  manager_.updateMagFieldCalibration(samples);

  // Should detect the offset (approximately)
  EXPECT_NEAR(manager_.data().mag.hard_iron.x(), true_offset.x(), 0.1);
  EXPECT_NEAR(manager_.data().mag.hard_iron.y(), true_offset.y(), 0.1);
}

TEST_F(CalibrationManagerTest, UpdateMagFieldCalibration_InsufficientSamples) {
  std::vector<Eigen::Vector3d> samples;
  for (int i = 0; i < 50; ++i) {  // Only 50 samples
    samples.push_back(Eigen::Vector3d::Random());
  }

  // Should handle gracefully (no crash)
  manager_.updateMagFieldCalibration(samples);
}

// =================== Validation Tests ===================
TEST_F(CalibrationManagerTest, VersionCheck) {
  EXPECT_TRUE(manager_.checkVersion());

  manager_.data().version = "0.9.0";
  EXPECT_FALSE(manager_.checkVersion());
}

TEST_F(CalibrationManagerTest, IsStale_Fresh) {
  manager_.data().modified_time = std::time(nullptr);
  EXPECT_FALSE(manager_.isStale(365.0));  // Not stale within 1 year
}

TEST_F(CalibrationManagerTest, IsStale_Old) {
  manager_.data().modified_time = std::time(nullptr) - (400 * 24 * 3600);  // 400 days ago
  EXPECT_TRUE(manager_.isStale(365.0));  // Stale after 1 year
}

// =================== Save/Load Tests ===================
TEST_F(CalibrationManagerTest, SaveAndLoad) {
  // Set some non-default values
  manager_.data().vehicle_id = "TEST_VEHICLE_123";
  manager_.data().imu.accel_bias = Eigen::Vector3d(0.1, 0.2, 0.3);
  manager_.data().baro.field_offset_agl = 5.5;

  // Save to file
  std::string filepath = "/tmp/test_calib.json";
  EXPECT_TRUE(manager_.save(filepath));

  // Load into new manager
  CalibrationManager manager2;
  EXPECT_TRUE(manager2.load(filepath));

  // Verify loaded data (basic check - full impl would use JSON lib)
  EXPECT_EQ(manager2.data().version, CALIB_VERSION);
}

// =================== Checksum Tests ===================
TEST(CalibrationChecksumTest, ConsistentHash) {
  std::string data1 = "test_data_12345";
  std::string data2 = "test_data_12345";
  std::string data3 = "different_data";

  auto hash1 = CalibrationManager::computeChecksum(data1);
  auto hash2 = CalibrationManager::computeChecksum(data2);
  auto hash3 = CalibrationManager::computeChecksum(data3);

  EXPECT_EQ(hash1, hash2);  // Same data -> same hash
  EXPECT_NE(hash1, hash3);  // Different data -> different hash
}

// =================== Edge Cases ===================
TEST_F(CalibrationManagerTest, ApplyLidar_ZeroPitchAngle) {
  manager_.data().lidar.pitch_angle_deg = 0.0;

  double raw_range = 10.0;
  auto corrected = manager_.applyLidar(raw_range);

  EXPECT_NEAR(corrected, raw_range, 1e-6);  // No correction
}

TEST_F(CalibrationManagerTest, ApplyImuGyro_ZeroTemperature) {
  Eigen::Vector3d raw_gyro(0.1, 0.2, 0.3);
  auto corrected = manager_.applyImuGyro(raw_gyro, 0.0);

  // Should still work with zero temperature
  EXPECT_TRUE(corrected.allFinite());
}
