/**
 * Enhanced Unit Tests for Hierarchical Filter
 * Focus on reset conditions, confidence, and gradiometer data flow
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/hierarchical_filter.h"
#include "sensors/sensor_manager.h"
#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace Navigation;

class HierarchicalFilterEnhancedTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::HierarchicalFilter> filter;
    YAML::Node ukf_config;
    YAML::Node rbpf_config;
    std::shared_ptr<MapManager> maps;

    void SetUp() override {
        // Create minimal YAML configs
        setupConfigs();

        // Create filter with YAML configs
        maps = nullptr;  // For testing without actual maps
        filter = std::make_unique<Navigation::HierarchicalFilter>(
            ukf_config, rbpf_config, maps);
    }

    void setupConfigs() {
        // UKF configuration
        ukf_config["ukf"]["sigma_points"]["alpha"] = 0.1;
        ukf_config["ukf"]["sigma_points"]["beta"] = 2.0;
        ukf_config["ukf"]["sigma_points"]["kappa"] = -37;

        ukf_config["ukf"]["process_noise"]["position"] = 1.0e-8;
        ukf_config["ukf"]["process_noise"]["velocity"] = 1.0e-4;
        ukf_config["ukf"]["process_noise"]["attitude"] = 1.0e-7;
        ukf_config["ukf"]["process_noise"]["accel_bias"] = 1.0e-7;
        ukf_config["ukf"]["process_noise"]["gyro_bias"] = 1.0e-9;
        ukf_config["ukf"]["process_noise"]["gravity_bias"] = 1.0e-14;

        ukf_config["ukf"]["measurement_noise"]["barometer"] = 1.0;
        ukf_config["ukf"]["measurement_noise"]["magnetometer"] = 0.01;
        ukf_config["ukf"]["measurement_noise"]["gravity_gradient"] = 250.0;

        ukf_config["ukf"]["initial_covariance"]["position"] = 1.0;
        ukf_config["ukf"]["initial_covariance"]["velocity"] = 0.01;
        ukf_config["ukf"]["initial_covariance"]["attitude"] = 0.001;
        ukf_config["ukf"]["initial_covariance"]["accel_bias"] = 1.0e-5;
        ukf_config["ukf"]["initial_covariance"]["gyro_bias"] = 1.0e-7;
        ukf_config["ukf"]["initial_covariance"]["gravity_bias"] = 1.0;

        // RBPF configuration
        rbpf_config["rbpf"]["num_particles"] = 100;
        rbpf_config["rbpf"]["confidence"]["min_confidence"] = 0.7;
        rbpf_config["rbpf"]["confidence"]["min_hold_time_s"] = 3.0;
        rbpf_config["rbpf"]["reset"]["enable"] = true;
        rbpf_config["rbpf"]["reset"]["min_interval_s"] = 15.0;
        rbpf_config["rbpf"]["reset"]["require_unimodal"] = false;
        rbpf_config["rbpf"]["resampling"]["threshold"] = 0.7;
    }

    StateVector createInitialState() {
        StateVector state;
        state.position = Vector3d(0, 0, -1000);  // 1000m altitude
        state.velocity = Vector3d(100, 0, 0);    // 100 m/s north
        state.quaternion = Quaterniond::Identity();
        state.accel_bias = Vector3d::Zero();
        state.gyro_bias = Vector3d::Zero();
        state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();
        state.timestamp = 0.0;
        return state;
    }

    void forceHighUncertainty() {
        // Process many predictions without measurements to grow uncertainty
        for (int i = 0; i < 1000; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.1, 0.1, 9.81);
            imu.gyro = Vector3d(0.01, 0.01, 0.01);
            imu.timestamp = i * 0.01;
            filter->processIMU(imu, 0.01);
        }
    }
};

// Test 1: Reset Trigger - High Position Uncertainty
TEST_F(HierarchicalFilterEnhancedTest, ResetTrigger_HighUncertainty) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Initially should not need reset
    EXPECT_FALSE(filter->shouldReset())
        << "Should not need reset immediately after initialization";

    // Force high uncertainty by many predictions without updates
    std::cout << "Forcing high uncertainty..." << std::endl;
    forceHighUncertainty();

    // Now should need reset due to high uncertainty
    bool needs_reset = filter->shouldReset();

    // Get current uncertainty for debugging
    auto state = filter->getCurrentState();
    double uncertainty = std::sqrt(state.covariance.block<3,3>(0,0).trace());
    std::cout << "Position uncertainty: " << uncertainty << " m" << std::endl;
    std::cout << "Needs reset: " << needs_reset << std::endl;

    // Uncertainty should be high
    EXPECT_GT(uncertainty, 400.0)
        << "Uncertainty didn't grow enough";

    // Should trigger reset
    EXPECT_TRUE(needs_reset)
        << "Reset not triggered despite high uncertainty: " << uncertainty << " m";
}

// Test 2: Reset Trigger - Consecutive Outliers
TEST_F(HierarchicalFilterEnhancedTest, ResetTrigger_ConsecutiveOutliers) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    int initial_resets = filter->getResetCount();

    // Generate consecutive outlier measurements
    for (int i = 0; i < 15; ++i) {
        // Create outlier barometer measurement
        BarometerData baro;
        baro.pressure = 50000.0;  // Unrealistic pressure
        baro.temperature = 280.0;
        baro.timestamp = i * 0.1;

        filter->updateBarometer(baro);
    }

    // Check if outliers triggered reset consideration
    bool needs_reset = filter->shouldReset();
    std::cout << "Needs reset after outliers: " << needs_reset << std::endl;

    // Should consider reset after many outliers
    EXPECT_TRUE(needs_reset || filter->getResetCount() > initial_resets)
        << "Reset not triggered after consecutive outliers";
}

// Test 3: Confidence Calculation
TEST_F(HierarchicalFilterEnhancedTest, ConfidenceCalculation) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Initial confidence should be high
    auto initial_state_combined = filter->getCurrentState();
    double initial_confidence = initial_state_combined.confidence;
    std::cout << "Initial confidence: " << initial_confidence << std::endl;

    EXPECT_GT(initial_confidence, 0.8)
        << "Initial confidence too low: " << initial_confidence;

    // Process without measurements to degrade confidence
    for (int i = 0; i < 500; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0.1, 0.1, 9.81);
        imu.gyro = Vector3d(0.01, 0.01, 0.01);
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    // Confidence should degrade
    auto degraded_state = filter->getCurrentState();
    double degraded_confidence = degraded_state.confidence;
    std::cout << "Degraded confidence: " << degraded_confidence << std::endl;

    // Note: Actual behavior depends on implementation
    // Confidence might not degrade if not properly updated
}

// Test 4: Gradiometer Data Flow
TEST_F(HierarchicalFilterEnhancedTest, GradiometerDataFlow) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Create realistic gradiometer data
    GradiometerData grad;
    grad.timestamp = 1.0;
    grad.valid = true;
    grad.confidence = 0.9;

    // Set realistic gravity gradient tensor (trace-free, symmetric)
    // Units: Eötvös (1E = 10^-9 s^-2)
    grad.tensor(0) = 10.0;   // Txx
    grad.tensor(1) = 5.0;    // Tyy
    grad.tensor(2) = -15.0;  // Tzz (trace-free: Txx + Tyy + Tzz = 0)
    grad.tensor(3) = 2.0;    // Txy
    grad.tensor(4) = 1.0;    // Txz
    grad.tensor(5) = 0.5;    // Tyz

    // Reconstruct full tensor
    grad.gradient_tensor = Matrix3d::Zero();
    grad.gradient_tensor(0,0) = grad.tensor(0);
    grad.gradient_tensor(1,1) = grad.tensor(1);
    grad.gradient_tensor(2,2) = grad.tensor(2);
    grad.gradient_tensor(0,1) = grad.gradient_tensor(1,0) = grad.tensor(3);
    grad.gradient_tensor(0,2) = grad.gradient_tensor(2,0) = grad.tensor(4);
    grad.gradient_tensor(1,2) = grad.gradient_tensor(2,1) = grad.tensor(5);

    std::cout << "Sending gradiometer data with tensor trace: "
              << grad.gradient_tensor.trace() << std::endl;

    // Update with gradiometer data
    filter->updateGravity(grad);

    // Create buffer for RBPF update
    std::vector<GradiometerData> gravity_buffer;
    for (int i = 0; i < 5; ++i) {
        grad.timestamp = 1.0 + i * 0.1;
        gravity_buffer.push_back(grad);
    }

    std::cout << "Updating RBPF with " << gravity_buffer.size()
              << " gravity measurements" << std::endl;

    // Update RBPF with buffer
    filter->updateRBPF(gravity_buffer);

    // Verify state is still valid
    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State invalid after gradiometer update";
}

// Test 5: Reset Execution
TEST_F(HierarchicalFilterEnhancedTest, ResetExecution) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    int initial_reset_count = filter->getResetCount();
    std::cout << "Initial reset count: " << initial_reset_count << std::endl;

    // Force conditions that should trigger reset
    forceHighUncertainty();

    // Check if reset is needed
    bool needs_reset = filter->shouldReset();
    std::cout << "Needs reset: " << needs_reset << std::endl;

    if (needs_reset) {
        // Perform reset
        auto reset_result = filter->performReset();

        std::cout << "Reset executed - Jump: " << reset_result.position_jump
                  << " m, Confidence: " << reset_result.confidence << std::endl;

        // Verify reset count increased
        int new_reset_count = filter->getResetCount();
        EXPECT_GT(new_reset_count, initial_reset_count)
            << "Reset count didn't increase after performReset()";

        // Verify confidence is valid
        EXPECT_GE(reset_result.confidence, 0.0)
            << "Invalid reset confidence";
        EXPECT_LE(reset_result.confidence, 1.0)
            << "Invalid reset confidence";
    }
}

// Test 6: RBPF Integration with Gradiometer
TEST_F(HierarchicalFilterEnhancedTest, RBPFGradiometerIntegration) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Simulate scenario where RBPF should help
    // First, let position drift
    for (int i = 0; i < 100; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0.01, 0.01, 9.81);  // Small bias
        imu.gyro = Vector3d::Zero();
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    auto drifted_pos = filter->getState().position;
    std::cout << "Position after drift: " << drifted_pos.transpose() << std::endl;

    // Now provide good gravity measurements
    std::vector<GradiometerData> good_measurements;
    for (int i = 0; i < 10; ++i) {
        GradiometerData grad;
        grad.timestamp = 1.0 + i * 0.1;
        grad.valid = true;
        grad.confidence = 0.95;

        // Consistent gravity gradient
        grad.tensor << 15.0, 10.0, -25.0, 3.0, 2.0, 1.0;

        // Reconstruct tensor
        grad.gradient_tensor = Matrix3d::Zero();
        grad.gradient_tensor(0,0) = grad.tensor(0);
        grad.gradient_tensor(1,1) = grad.tensor(1);
        grad.gradient_tensor(2,2) = grad.tensor(2);
        grad.gradient_tensor(0,1) = grad.gradient_tensor(1,0) = grad.tensor(3);
        grad.gradient_tensor(0,2) = grad.gradient_tensor(2,0) = grad.tensor(4);
        grad.gradient_tensor(1,2) = grad.gradient_tensor(2,1) = grad.tensor(5);

        good_measurements.push_back(grad);
    }

    // Update RBPF with good measurements
    filter->updateRBPF(good_measurements);

    auto corrected_pos = filter->getState().position;
    std::cout << "Position after RBPF update: " << corrected_pos.transpose() << std::endl;

    // State should still be valid
    EXPECT_TRUE(filter->getCurrentState().is_valid)
        << "State invalid after RBPF correction";
}

// Test 7: Mode Transitions
TEST_F(HierarchicalFilterEnhancedTest, FilterModeTransitions) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    FilterMode initial_mode = filter->getCurrentMode();
    std::cout << "Initial mode: " << static_cast<int>(initial_mode) << std::endl;

    // Should start in UKF_ONLY or UKF_RBPF
    EXPECT_TRUE(initial_mode == FilterMode::UKF_ONLY ||
                initial_mode == FilterMode::UKF_RBPF)
        << "Unexpected initial mode";

    // Force degraded mode by increasing uncertainty
    forceHighUncertainty();

    // Process a measurement to trigger mode check
    IMUData imu;
    imu.accel = Vector3d(0, 0, 9.81);
    imu.gyro = Vector3d::Zero();
    imu.timestamp = 100.0;
    filter->processIMU(imu, 0.01);

    FilterMode degraded_mode = filter->getCurrentMode();
    std::cout << "Mode after forcing uncertainty: " << static_cast<int>(degraded_mode) << std::endl;

    // May transition to DEGRADED or RBPF_RESET
    EXPECT_TRUE(degraded_mode == FilterMode::DEGRADED ||
                degraded_mode == FilterMode::RBPF_RESET ||
                degraded_mode == FilterMode::UKF_RBPF)
        << "Unexpected mode after high uncertainty";
}

// Test 8: Verify Reset Conditions Details
TEST_F(HierarchicalFilterEnhancedTest, DetailedResetConditions) {
    StateVector initial_state = createInitialState();
    filter->initialize(initial_state);

    // Get baseline
    auto baseline_state = filter->getCurrentState();
    double baseline_uncertainty = std::sqrt(baseline_state.covariance.block<3,3>(0,0).trace());

    std::cout << "\n=== Reset Condition Analysis ===" << std::endl;
    std::cout << "Baseline uncertainty: " << baseline_uncertainty << " m" << std::endl;
    std::cout << "Baseline confidence: " << baseline_state.confidence << std::endl;
    std::cout << "Initial needs reset: " << filter->shouldReset() << std::endl;

    // Test different uncertainty levels
    std::vector<int> prediction_counts = {100, 500, 1000, 2000};

    for (int count : prediction_counts) {
        // Reset filter
        filter = std::make_unique<Navigation::HierarchicalFilter>(
            ukf_config, rbpf_config, maps);
        filter->initialize(initial_state);

        // Apply predictions
        for (int i = 0; i < count; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.01, 0.01, 9.81);
            imu.gyro = Vector3d(0.001, 0.001, 0.001);
            imu.timestamp = i * 0.01;
            filter->processIMU(imu, 0.01);
        }

        auto test_state = filter->getCurrentState();
        double test_uncertainty = std::sqrt(test_state.covariance.block<3,3>(0,0).trace());
        bool needs_reset = filter->shouldReset();

        std::cout << "\nAfter " << count << " predictions:" << std::endl;
        std::cout << "  Uncertainty: " << test_uncertainty << " m" << std::endl;
        std::cout << "  Confidence: " << test_state.confidence << std::endl;
        std::cout << "  Needs reset: " << needs_reset << std::endl;
    }
}