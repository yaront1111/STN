/**
 * Unit Tests for Hierarchical Filter
 * Tests filter orchestration and mode transitions
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/hierarchical_filter.h"
#include "sensors/sensor_manager.h"

using namespace Eigen;
using namespace Navigation;

class HierarchicalFilterTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::HierarchicalFilter> filter;
    HierarchicalConfig config;
    std::shared_ptr<MapManager> maps;

    void SetUp() override {
        // Configure hierarchical filter
        config.ukf_rate = 100.0;
        config.rbpf_rate = 1.0;
        config.enable_adaptive_rate = true;
        config.enable_hard_reset = true;
        config.ukf_weight = 0.7;
        config.map_weight = 0.3;

        // Configure reset triggers
        config.reset_triggers.position_uncertainty_threshold = 500.0;
        config.reset_triggers.nees_threshold = 15.0;
        config.reset_triggers.nis_threshold = 12.0;
        config.reset_triggers.consecutive_outliers = 10;
        config.reset_triggers.time_since_last_fix = 300.0;

        // Note: In real usage, would need actual map managers
        // For testing, we can use nullptr and test will still work
        maps = nullptr;

        filter = std::make_unique<Navigation::HierarchicalFilter>(config, maps);
    }

    StateVector createInitialState() {
        StateVector state;
        state.position = Vector3d(0, 0, -1000);  // 1000m altitude
        state.velocity = Vector3d(10, 0, 0);     // Moving north at 10 m/s
        state.quaternion = Quaterniond::Identity();
        state.accel_bias = Vector3d::Zero();
        state.gyro_bias = Vector3d::Zero();
        return state;
    }
};

// Test initialization
TEST_F(HierarchicalFilterTest, Initialization) {
    StateVector initial_state = createInitialState();

    // Test simplified initialization
    EXPECT_TRUE(filter->initialize(initial_state))
        << "Failed to initialize filter";

    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State not valid after initialization";

    EXPECT_LT((state.position - initial_state.position).norm(), 1e-6)
        << "Position not correctly initialized";
    EXPECT_LT((state.velocity - initial_state.velocity).norm(), 1e-6)
        << "Velocity not correctly initialized";
}

// Test IMU processing
TEST_F(HierarchicalFilterTest, IMUProcessing) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create IMU data
    IMUData imu;
    imu.accel = Vector3d(0, 0, 9.81);  // Gravity
    imu.gyro = Vector3d(0, 0, 0);      // No rotation
    imu.timestamp = 0.01;

    double dt = 0.01;

    // Process IMU
    auto result = filter->processIMU(imu, dt);

    EXPECT_TRUE(result.is_valid)
        << "Result not valid after IMU processing";

    // Position should have changed based on velocity
    Vector3d expected_pos = initial_state.position + initial_state.velocity * dt;
    EXPECT_LT((result.position - expected_pos).norm(), 0.1)
        << "Position prediction incorrect";
}

// Test barometer update
TEST_F(HierarchicalFilterTest, BarometerUpdate) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create barometer data
    BarometerData baro;
    baro.pressure = 90000.0;     // Lower pressure at altitude
    baro.temperature = 280.0;     // Kelvin
    baro.timestamp = 0.01;

    // Update with barometer
    filter->updateBarometer(baro);

    auto state = filter->getCurrentState();

    // Altitude should be refined
    EXPECT_TRUE(state.is_valid)
        << "State not valid after barometer update";
}

// Test magnetometer update
TEST_F(HierarchicalFilterTest, MagnetometerUpdate) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create magnetometer data
    MagnetometerData mag;
    mag.field = Vector3d(30e-6, 0, 40e-6);  // Typical Earth field in Tesla
    mag.timestamp = 0.01;

    // Update with magnetometer
    filter->updateMagnetometer(mag);

    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State not valid after magnetometer update";

    // Attitude should be influenced by magnetic field
    // (exact behavior depends on implementation)
}

// Test gravity gradiometer update
TEST_F(HierarchicalFilterTest, GravityUpdate) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create gradiometer data
    GradiometerData grad;
    grad.gradient_tensor = Matrix3d::Identity() * 0.1;  // Small gradient
    grad.timestamp = 0.01;

    // Update with gravity
    filter->updateGravity(grad);

    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State not valid after gravity update";
}

// Test ML bias update
TEST_F(HierarchicalFilterTest, MLBiasUpdate) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Apply ML-predicted bias
    Vector3d bias_pred(0.01, 0.02, 0.03);
    double uncertainty = 0.1;

    filter->updateMLBias(bias_pred, uncertainty);

    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State not valid after ML bias update";
}

// Test filter mode transitions
TEST_F(HierarchicalFilterTest, ModeTransitions) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Initially should be in UKF_ONLY or UKF_RBPF mode
    FilterMode initial_mode = filter->getCurrentMode();
    EXPECT_TRUE(initial_mode == FilterMode::UKF_ONLY ||
                initial_mode == FilterMode::UKF_RBPF)
        << "Unexpected initial filter mode";
}

// Test reset conditions
TEST_F(HierarchicalFilterTest, ResetConditions) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Initially shouldn't need reset
    EXPECT_FALSE(filter->shouldReset())
        << "Filter shouldn't need reset immediately after initialization";

    EXPECT_EQ(filter->getResetCount(), 0)
        << "Reset count should be zero initially";
}

// Test combined state processing
TEST_F(HierarchicalFilterTest, CombinedProcessing) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create sensor data
    SensorData sensor_data;

    // Add IMU data
    IMUData imu;
    imu.accel = Vector3d(0.1, 0.2, 9.81);
    imu.gyro = Vector3d(0.01, 0.02, 0.03);
    imu.timestamp = 0.01;
    sensor_data.imu = imu;

    // Add barometer data
    BarometerData baro;
    baro.pressure = 101325.0;
    baro.temperature = 288.15;
    baro.timestamp = 0.01;
    sensor_data.barometer = baro;

    double dt = 0.01;

    // Process combined sensor data
    auto result = filter->orchestrateFilters(sensor_data, dt);

    EXPECT_TRUE(result.is_valid)
        << "Result not valid after combined processing";

    // Check that all components are reasonable
    EXPECT_TRUE(result.position.allFinite())
        << "Position contains non-finite values";
    EXPECT_TRUE(result.velocity.allFinite())
        << "Velocity contains non-finite values";
    EXPECT_NEAR(result.attitude.norm(), 1.0, 1e-6)
        << "Attitude quaternion not normalized";
}

// Test state getter methods
TEST_F(HierarchicalFilterTest, StateGetters) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Test state getter
    StateVector state = filter->getState();
    EXPECT_LT((state.position - initial_state.position).norm(), 1e-6)
        << "State getter returns wrong position";

    // Test covariance getter
    auto cov = filter->getCovariance();
    EXPECT_EQ(cov.rows(), 21)
        << "Covariance has wrong dimensions";
    EXPECT_EQ(cov.cols(), 21)
        << "Covariance has wrong dimensions";

    // Check covariance is symmetric
    EXPECT_LT((cov - cov.transpose()).norm(), 1e-10)
        << "Covariance not symmetric";
}

// Test multiple IMU updates
TEST_F(HierarchicalFilterTest, MultipleIMUUpdates) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Process multiple IMU samples
    for (int i = 0; i < 100; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0, 0, 9.81);
        imu.gyro = Vector3d(0, 0, 0.1);  // Slow yaw rotation
        imu.timestamp = i * 0.01;

        auto result = filter->processIMU(imu, 0.01);
        EXPECT_TRUE(result.is_valid)
            << "Result invalid at step " << i;
    }

    auto final_state = filter->getCurrentState();

    // After rotating for 1 second at 0.1 rad/s, should have rotated ~0.1 rad
    auto euler = final_state.attitude.toRotationMatrix().eulerAngles(0, 1, 2);
    EXPECT_NEAR(std::abs(euler(2)), 0.1, 0.02)
        << "Rotation integration incorrect";
}

// Test filter confidence
TEST_F(HierarchicalFilterTest, FilterConfidence) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    auto state = filter->getCurrentState();

    // Confidence should be between 0 and 1
    EXPECT_GE(state.confidence, 0.0)
        << "Confidence below 0";
    EXPECT_LE(state.confidence, 1.0)
        << "Confidence above 1";
}

// Test processing with missing sensor data
TEST_F(HierarchicalFilterTest, MissingSensorData) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create sensor data with only some fields populated
    SensorData partial_data;

    // Only IMU data available
    IMUData imu;
    imu.accel = Vector3d(0, 0, 9.81);
    imu.gyro = Vector3d::Zero();
    imu.timestamp = 0.01;
    partial_data.imu = imu;

    // No barometer or magnetometer data

    double dt = 0.01;

    // Should still process available data
    auto result = filter->orchestrateFilters(partial_data, dt);
    EXPECT_TRUE(result.is_valid)
        << "Failed to process with partial sensor data";
}

// Test RBPF buffer update
TEST_F(HierarchicalFilterTest, RBPFBufferUpdate) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create a buffer of gravity measurements
    std::vector<GradiometerData> gravity_buffer;
    for (int i = 0; i < 10; ++i) {
        GradiometerData grad;
        grad.gradient_tensor = Matrix3d::Identity() * (0.1 + i * 0.01);
        grad.timestamp = i * 0.1;
        gravity_buffer.push_back(grad);
    }

    // Update RBPF with buffer
    filter->updateRBPF(gravity_buffer);

    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State not valid after RBPF buffer update";
}

// Test extreme values handling
TEST_F(HierarchicalFilterTest, ExtremeValues) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Test with very large acceleration
    IMUData extreme_imu;
    extreme_imu.accel = Vector3d(1000, 1000, 1000);  // Unrealistic
    extreme_imu.gyro = Vector3d(50, 50, 50);         // Unrealistic
    extreme_imu.timestamp = 0.01;

    // Should handle gracefully (either reject or cap)
    auto result = filter->processIMU(extreme_imu, 0.01);

    // Check state remains reasonable
    EXPECT_LT(result.position.norm(), 1e6)
        << "Position exploded with extreme inputs";
    EXPECT_LT(result.velocity.norm(), 1e4)
        << "Velocity exploded with extreme inputs";
}

// Test reset execution
TEST_F(HierarchicalFilterTest, ResetExecution) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Get initial reset count
    int initial_resets = filter->getResetCount();

    // Force a reset condition and execute
    // Note: In real scenario, this would be triggered by poor performance
    if (config.enable_hard_reset) {
        auto reset_result = filter->performReset();

        // Check reset was executed
        EXPECT_EQ(filter->getResetCount(), initial_resets + 1)
            << "Reset count didn't increment";

        // Check reset result
        EXPECT_GE(reset_result.confidence, 0.0)
            << "Reset confidence invalid";
        EXPECT_LE(reset_result.confidence, 1.0)
            << "Reset confidence invalid";
    }
}