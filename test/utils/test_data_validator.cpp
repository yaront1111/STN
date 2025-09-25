/**
 * Unit Tests for Data Validation
 * Ensures sensor data integrity and sanity checking
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "utils/data_validator.h"
#include "sensors/sensor_manager.h"

using namespace Eigen;
using namespace Navigation;

class DataValidatorTest : public ::testing::Test {
protected:
    std::unique_ptr<DataValidator> validator;

    void SetUp() override {
        // Initialize validator with reasonable thresholds
        DataValidatorConfig config;
        config.max_accel = 200.0;      // 20G max
        config.max_gyro = 10.0;         // 10 rad/s max
        config.max_mag_field = 100e-6;  // 100 uT max (Tesla)
        config.min_pressure = 10000.0;  // 10 kPa min
        config.max_pressure = 120000.0; // 120 kPa max
        config.max_position_innovation = 100.0; // 100m max jump
        config.max_velocity_innovation = 10.0;  // 10 m/s max

        validator = std::make_unique<DataValidator>(config);
    }
};

// Test IMU data validation
TEST_F(DataValidatorTest, IMUDataValidation) {
    // Valid IMU data
    IMUData imu_data;
    imu_data.accel = Vector3d(0, 0, 9.81);
    imu_data.gyro = Vector3d(0.1, 0.2, 0.3);
    imu_data.timestamp = 1.0;

    auto result = validator->validateIMU(imu_data);
    EXPECT_TRUE(result.valid)
        << "Valid IMU data rejected: " << result.failure_reason;

    // Invalid acceleration (too large)
    IMUData bad_accel_data;
    bad_accel_data.accel = Vector3d(300, 0, 0);  // 30G
    bad_accel_data.gyro = Vector3d(0, 0, 0);
    bad_accel_data.timestamp = 1.01;

    result = validator->validateIMU(bad_accel_data);
    EXPECT_FALSE(result.valid)
        << "Excessive acceleration not detected";

    // Invalid gyro (NaN)
    IMUData bad_gyro_data;
    bad_gyro_data.accel = Vector3d(0, 0, 9.81);
    bad_gyro_data.gyro = Vector3d(0, NAN, 0);
    bad_gyro_data.timestamp = 1.02;

    result = validator->validateIMU(bad_gyro_data);
    EXPECT_FALSE(result.valid)
        << "NaN in gyro not detected";

    // Edge case: exactly at threshold
    IMUData edge_data;
    edge_data.accel = Vector3d(200, 0, 0);
    edge_data.gyro = Vector3d(10, 0, 0);
    edge_data.timestamp = 1.03;

    result = validator->validateIMU(edge_data);
    EXPECT_TRUE(result.valid)
        << "Data at threshold rejected";
}

// Test barometer validation
TEST_F(DataValidatorTest, BarometerValidation) {
    // Sea level pressure
    BarometerData baro_data;
    baro_data.pressure = 101325.0;     // Pa
    baro_data.temperature = 288.15;    // K
    baro_data.timestamp = 1.0;

    auto result = validator->validateBarometer(baro_data);
    EXPECT_TRUE(result.valid)
        << "Valid barometer data rejected: " << result.failure_reason;

    // Invalid pressure (too low - space)
    BarometerData low_pressure_data;
    low_pressure_data.pressure = 100.0;  // Too low
    low_pressure_data.temperature = 288.15;
    low_pressure_data.timestamp = 1.01;

    result = validator->validateBarometer(low_pressure_data);
    EXPECT_FALSE(result.valid)
        << "Space-level pressure not detected";

    // Invalid temperature (below reasonable range)
    BarometerData cold_data;
    cold_data.pressure = 101325.0;
    cold_data.temperature = -100.0;  // -100°C is unreasonable for aircraft
    cold_data.timestamp = 1.02;

    result = validator->validateBarometer(cold_data);
    // Temperature out of range should generate warning but not invalidate
    EXPECT_TRUE(result.valid)  // Pressure is valid
        << "Valid pressure rejected due to temperature";

    // High altitude pressure (should be valid)
    BarometerData high_alt_data;
    high_alt_data.pressure = 20000.0;  // ~12km altitude
    high_alt_data.temperature = 220.0;
    high_alt_data.timestamp = 1.03;

    result = validator->validateBarometer(high_alt_data);
    EXPECT_TRUE(result.valid)
        << "High altitude pressure wrongly rejected";
}

// Test magnetometer validation
TEST_F(DataValidatorTest, MagnetometerValidation) {
    // Typical Earth field (30-60 uT in Tesla)
    MagnetometerData mag_data;
    mag_data.field = Vector3d(30e-6, 0, 40e-6);  // 50 uT total in Tesla
    mag_data.timestamp = 1.0;

    auto result = validator->validateMagnetometer(mag_data);
    EXPECT_TRUE(result.valid)
        << "Valid magnetic field rejected: " << result.failure_reason;

    // No field (sensor failure)
    MagnetometerData zero_data;
    zero_data.field = Vector3d(0, 0, 0);
    zero_data.timestamp = 1.01;

    result = validator->validateMagnetometer(zero_data);
    EXPECT_FALSE(result.valid)
        << "Zero magnetic field not detected";

    // Excessive field (near magnet)
    MagnetometerData strong_data;
    strong_data.field = Vector3d(200e-6, 300e-6, 400e-6);  // Way too strong in Tesla
    strong_data.timestamp = 1.02;

    result = validator->validateMagnetometer(strong_data);
    EXPECT_FALSE(result.valid)
        << "Excessive magnetic field not detected";

    // Check for infinity
    MagnetometerData inf_data;
    inf_data.field = Vector3d(30e-6, INFINITY, 40e-6);
    inf_data.timestamp = 1.03;

    result = validator->validateMagnetometer(inf_data);
    EXPECT_FALSE(result.valid)
        << "Infinity in magnetic field not detected";
}

// Test finite value checking
TEST_F(DataValidatorTest, FiniteValueCheck) {
    VectorXd valid_data(3);
    valid_data << 1.0, 2.0, 3.0;
    EXPECT_TRUE(validator->checkFinite(valid_data))
        << "Valid data marked as non-finite";

    VectorXd nan_data(3);
    nan_data << 1.0, NAN, 3.0;
    EXPECT_FALSE(validator->checkFinite(nan_data))
        << "NaN not detected";

    VectorXd inf_data(3);
    inf_data << 1.0, 2.0, INFINITY;
    EXPECT_FALSE(validator->checkFinite(inf_data))
        << "Infinity not detected";
}

// Test range checking
TEST_F(DataValidatorTest, RangeCheck) {
    EXPECT_TRUE(validator->checkRange(5.0, 0.0, 10.0))
        << "Value in range rejected";

    EXPECT_FALSE(validator->checkRange(-1.0, 0.0, 10.0))
        << "Value below range not detected";

    EXPECT_FALSE(validator->checkRange(11.0, 0.0, 10.0))
        << "Value above range not detected";

    // Edge cases
    EXPECT_TRUE(validator->checkRange(0.0, 0.0, 10.0))
        << "Lower bound rejected";
    EXPECT_TRUE(validator->checkRange(10.0, 0.0, 10.0))
        << "Upper bound rejected";
}

// Test vector magnitude checking
TEST_F(DataValidatorTest, VectorMagnitudeCheck) {
    Vector3d small_vec(1, 2, 2);  // Magnitude = 3
    EXPECT_TRUE(validator->checkVectorMagnitude(small_vec, 5.0))
        << "Small vector rejected";

    Vector3d large_vec(3, 4, 0);  // Magnitude = 5
    EXPECT_FALSE(validator->checkVectorMagnitude(large_vec, 4.0))
        << "Large vector not detected";

    // Edge case
    Vector3d edge_vec(0, 0, 5);  // Magnitude = 5
    EXPECT_TRUE(validator->checkVectorMagnitude(edge_vec, 5.0))
        << "Vector at threshold rejected";
}

// Test timestamp validation
TEST_F(DataValidatorTest, TimestampValidation) {
    // Normal time gap (10ms)
    EXPECT_TRUE(validator->checkTimestamp(1.01, 1.00))
        << "Normal timestamp gap rejected";

    // Too large gap (>1s)
    EXPECT_FALSE(validator->checkTimestamp(3.0, 1.0))
        << "Large time gap not detected";

    // Negative time (going backwards)
    EXPECT_FALSE(validator->checkTimestamp(0.9, 1.0))
        << "Backward time not detected";

    // Too small gap (<1ms)
    EXPECT_FALSE(validator->checkTimestamp(1.0001, 1.0))
        << "Too small time gap not detected";
}

// Test consistency checking
TEST_F(DataValidatorTest, ConsistencyCheck) {
    Vector3d prev(1, 2, 3);

    // Small change
    Vector3d curr1(1.5, 2.5, 3.5);
    EXPECT_TRUE(validator->checkConsistency(curr1, prev, 1.0))
        << "Small consistent change rejected";

    // Large change
    Vector3d curr2(10, 20, 30);
    EXPECT_FALSE(validator->checkConsistency(curr2, prev, 1.0))
        << "Large inconsistent change not detected";

    // No change
    EXPECT_TRUE(validator->checkConsistency(prev, prev, 1.0))
        << "No change rejected";
}

// Test innovation validation
TEST_F(DataValidatorTest, InnovationValidation) {
    // Small position innovation
    Vector3d small_pos_innovation(1, 2, 3);
    EXPECT_TRUE(validator->validatePositionInnovation(small_pos_innovation))
        << "Small position innovation rejected";

    // Large position innovation
    Vector3d large_pos_innovation(150, 0, 0);
    EXPECT_FALSE(validator->validatePositionInnovation(large_pos_innovation))
        << "Large position innovation not detected";

    // Small velocity innovation
    Vector3d small_vel_innovation(0.5, 0.5, 0.5);
    EXPECT_TRUE(validator->validateVelocityInnovation(small_vel_innovation))
        << "Small velocity innovation rejected";

    // Large velocity innovation
    Vector3d large_vel_innovation(15, 0, 0);
    EXPECT_FALSE(validator->validateVelocityInnovation(large_vel_innovation))
        << "Large velocity innovation not detected";

    // Small attitude innovation
    Vector3d small_att_innovation(0.01, 0.01, 0.01);
    EXPECT_TRUE(validator->validateAttitudeInnovation(small_att_innovation))
        << "Small attitude innovation rejected";

    // Large attitude innovation
    Vector3d large_att_innovation(1.0, 0, 0);
    EXPECT_FALSE(validator->validateAttitudeInnovation(large_att_innovation))
        << "Large attitude innovation not detected";
}

// Test statistics tracking
TEST_F(DataValidatorTest, StatisticsTracking) {
    auto initial_stats = validator->getStatistics();
    EXPECT_EQ(initial_stats.total_checks, 0)
        << "Non-zero initial checks";

    // Perform some validations
    IMUData imu;
    imu.accel = Vector3d(0, 0, 9.81);
    imu.gyro = Vector3d(0, 0, 0);
    imu.timestamp = 1.0;

    validator->validateIMU(imu);

    auto stats = validator->getStatistics();
    EXPECT_GT(stats.total_checks, 0)
        << "Statistics not updated";

    // Reset statistics
    validator->resetStatistics();
    stats = validator->getStatistics();
    EXPECT_EQ(stats.total_checks, 0)
        << "Statistics not reset";
}

// Test configuration update
TEST_F(DataValidatorTest, ConfigurationUpdate) {
    auto initial_config = validator->getConfig();

    // Update configuration
    DataValidatorConfig new_config;
    new_config.max_accel = 100.0;  // Reduce max acceleration
    new_config.max_gyro = 5.0;     // Reduce max gyro

    validator->updateConfig(new_config);

    // Test that new limits are enforced
    IMUData imu;
    imu.accel = Vector3d(150, 0, 0);  // Above new limit but below old
    imu.gyro = Vector3d(0, 0, 0);
    imu.timestamp = 1.0;

    auto result = validator->validateIMU(imu);
    EXPECT_FALSE(result.valid)
        << "New configuration not applied";
}