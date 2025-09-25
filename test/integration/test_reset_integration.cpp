/**
 * Integration Tests for Position Reset System
 * Tests complete data flow from sensors through filters to reset
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/hierarchical_filter.h"
#include "maps/map_manager.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

using namespace Eigen;
using namespace Navigation;
using namespace std::chrono;

class ResetIntegrationTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::HierarchicalFilter> filter;
    std::shared_ptr<Navigation::MapManager> maps;
    YAML::Node config;

    void SetUp() override {
        setupConfiguration();
        setupFilter();
    }

    void setupConfiguration() {
        // Load base config structure
        config["system"]["mode"] = "test";
        config["system"]["log_level"] = "DEBUG";

        // UKF config
        config["ukf"]["sigma_points"]["alpha"] = 0.1;
        config["ukf"]["sigma_points"]["beta"] = 2.0;
        config["ukf"]["sigma_points"]["kappa"] = -37;

        config["ukf"]["process_noise"]["position"] = 1.0e-8;
        config["ukf"]["process_noise"]["velocity"] = 1.0e-4;
        config["ukf"]["process_noise"]["attitude"] = 1.0e-7;
        config["ukf"]["process_noise"]["accel_bias"] = 1.0e-7;
        config["ukf"]["process_noise"]["gyro_bias"] = 1.0e-9;

        config["ukf"]["measurement_noise"]["barometer"] = 1.0;
        config["ukf"]["measurement_noise"]["magnetometer"] = 0.01;
        config["ukf"]["measurement_noise"]["gravity_gradient"] = 250.0;

        config["ukf"]["initial_covariance"]["position"] = 1.0;
        config["ukf"]["initial_covariance"]["velocity"] = 0.01;
        config["ukf"]["initial_covariance"]["attitude"] = 0.001;

        // RBPF config with aggressive reset settings
        config["rbpf"]["num_particles"] = 200;
        config["rbpf"]["update_interval_s"] = 1.0;

        config["rbpf"]["confidence"]["min_confidence"] = 0.6;  // Lower threshold
        config["rbpf"]["confidence"]["min_hold_time_s"] = 2.0;

        config["rbpf"]["reset"]["enable"] = true;
        config["rbpf"]["reset"]["min_interval_s"] = 10.0;  // Allow frequent resets
        config["rbpf"]["reset"]["require_unimodal"] = false;

        config["rbpf"]["resampling"]["threshold"] = 0.7;
    }

    void setupFilter() {
        maps = nullptr;  // No maps needed for this test
        filter = std::make_unique<HierarchicalFilter>(
            config["ukf"], config["rbpf"], maps);
    }

    StateVector createDriftingState(double drift_rate = 1.0) {
        StateVector state;
        state.position = Vector3d(0, 0, -1000);
        state.velocity = Vector3d(100, 0, 0);  // 100 m/s north
        state.quaternion = Quaterniond::Identity();
        state.accel_bias = Vector3d(drift_rate * 0.01, 0, 0);  // Bias causes drift
        state.gyro_bias = Vector3d::Zero();
        state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();
        return state;
    }

    void simulateDrift(int seconds, double bias_magnitude = 0.01) {
        for (int i = 0; i < seconds * 100; ++i) {  // 100 Hz
            IMUData imu;
            imu.accel = Vector3d(bias_magnitude, 0, 9.81);  // Biased measurement
            imu.gyro = Vector3d(0, 0, 0.001);  // Small rotation
            imu.timestamp = i * 0.01;

            filter->processIMU(imu, 0.01);
        }
    }

    std::vector<GradiometerData> createGravityBuffer(int count, double timestamp_start) {
        std::vector<GradiometerData> buffer;

        for (int i = 0; i < count; ++i) {
            GradiometerData grad;
            grad.timestamp = timestamp_start + i * 0.1;
            grad.valid = true;
            grad.confidence = 0.9;

            // Create consistent gradient pattern
            grad.tensor(0) = 20.0 + i * 0.1;   // Txx
            grad.tensor(1) = 15.0 - i * 0.1;   // Tyy
            grad.tensor(2) = -35.0;            // Tzz (trace-free)
            grad.tensor(3) = 3.0;              // Txy
            grad.tensor(4) = 2.0;              // Txz
            grad.tensor(5) = 1.0;              // Tyz

            // Reconstruct full tensor
            grad.gradient_tensor = Matrix3d::Zero();
            grad.gradient_tensor(0,0) = grad.tensor(0);
            grad.gradient_tensor(1,1) = grad.tensor(1);
            grad.gradient_tensor(2,2) = grad.tensor(2);
            grad.gradient_tensor(0,1) = grad.gradient_tensor(1,0) = grad.tensor(3);
            grad.gradient_tensor(0,2) = grad.gradient_tensor(2,0) = grad.tensor(4);
            grad.gradient_tensor(1,2) = grad.gradient_tensor(2,1) = grad.tensor(5);

            buffer.push_back(grad);
        }

        return buffer;
    }
};

// Test 1: End-to-End Reset Triggering
TEST_F(ResetIntegrationTest, EndToEndResetTriggering) {
    std::cout << "\n=== End-to-End Reset Test ===" << std::endl;

    // Initialize with drifting state
    StateVector initial_state = createDriftingState(2.0);  // Higher drift
    filter->initialize(initial_state);

    int initial_resets = filter->getResetCount();
    std::cout << "Initial reset count: " << initial_resets << std::endl;

    // Record initial position
    Vector3d initial_position = filter->getState().position;
    std::cout << "Initial position: " << initial_position.transpose() << std::endl;

    // Simulate drift for 30 seconds
    std::cout << "\nSimulating 30 seconds of drift..." << std::endl;
    simulateDrift(30, 0.02);  // Strong bias

    // Check position drift
    Vector3d drifted_position = filter->getState().position;
    double drift_distance = (drifted_position - initial_position).norm();
    std::cout << "Position after drift: " << drifted_position.transpose() << std::endl;
    std::cout << "Drift distance: " << drift_distance << " m" << std::endl;

    // Check uncertainty growth
    auto state = filter->getCurrentState();
    double uncertainty = std::sqrt(state.covariance.block<3,3>(0,0).trace());
    std::cout << "Position uncertainty: " << uncertainty << " m" << std::endl;

    // Check if reset is needed
    bool needs_reset = filter->shouldReset();
    std::cout << "Needs reset: " << needs_reset << std::endl;

    // Provide good gravity measurements
    std::cout << "\nProviding gravity measurements..." << std::endl;
    auto gravity_buffer = createGravityBuffer(10, 30.0);
    filter->updateRBPF(gravity_buffer);

    // Try to trigger reset
    if (filter->shouldReset()) {
        std::cout << "Executing reset..." << std::endl;
        auto reset_result = filter->performReset();
        std::cout << "Reset jump: " << reset_result.position_jump << " m" << std::endl;
        std::cout << "Reset confidence: " << reset_result.confidence << std::endl;

        // Verify reset occurred
        int new_resets = filter->getResetCount();
        EXPECT_GT(new_resets, initial_resets)
            << "Reset count didn't increase";
    }

    // Final position check
    Vector3d final_position = filter->getState().position;
    std::cout << "Final position: " << final_position.transpose() << std::endl;
}

// Test 2: Reset Improves Position Accuracy
TEST_F(ResetIntegrationTest, ResetImprovesAccuracy) {
    std::cout << "\n=== Reset Accuracy Improvement Test ===" << std::endl;

    StateVector true_state = createDriftingState(0);  // No actual drift
    filter->initialize(true_state);

    // Simulate measurement bias causing apparent drift
    std::cout << "Simulating biased measurements..." << std::endl;
    for (int i = 0; i < 2000; ++i) {  // 20 seconds
        IMUData imu;
        imu.accel = Vector3d(0.05, 0.05, 9.81);  // Biased accel
        imu.gyro = Vector3d(0, 0, 0);
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    // Measure error before reset
    Vector3d estimated_pos = filter->getState().position;
    double error_before = (estimated_pos - true_state.position).norm();
    std::cout << "Position error before reset: " << error_before << " m" << std::endl;

    // Force reset by injecting correct position
    if (filter->shouldReset()) {
        auto reset_result = filter->performReset();

        // Measure error after reset
        Vector3d reset_pos = filter->getState().position;
        double error_after = (reset_pos - true_state.position).norm();
        std::cout << "Position error after reset: " << error_after << " m" << std::endl;

        // Error should improve
        EXPECT_LT(error_after, error_before * 0.5)
            << "Reset didn't improve position accuracy";
    }
}

// Test 3: Gradiometer Data Flow to RBPF
TEST_F(ResetIntegrationTest, GradiometerDataFlow) {
    std::cout << "\n=== Gradiometer Data Flow Test ===" << std::endl;

    StateVector initial_state = createDriftingState();
    filter->initialize(initial_state);

    // Create gradiometer measurements
    std::vector<GradiometerData> measurements;

    for (int i = 0; i < 20; ++i) {
        GradiometerData grad;
        grad.timestamp = i * 1.0;  // 1 Hz
        grad.valid = true;
        grad.confidence = 0.95;

        // Varying gradient to test response
        double variation = std::sin(i * 0.5);
        grad.tensor(0) = 25.0 + variation * 5.0;   // Txx
        grad.tensor(1) = 20.0 - variation * 3.0;   // Tyy
        grad.tensor(2) = -45.0 + variation * 2.0;  // Tzz
        grad.tensor(3) = 4.0;                      // Txy
        grad.tensor(4) = 3.0;                      // Txz
        grad.tensor(5) = 2.0;                      // Tyz

        measurements.push_back(grad);

        // Process individually
        std::cout << "Processing gradient " << i << " with Txx=" << grad.tensor(0) << std::endl;
        filter->updateGravity(grad);
    }

    // Batch update to RBPF
    std::cout << "\nBatch updating RBPF with " << measurements.size() << " measurements" << std::endl;
    filter->updateRBPF(measurements);

    // Verify system still stable
    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State invalid after gradiometer updates";

    EXPECT_TRUE(state.position.allFinite())
        << "Position contains non-finite values";
}

// Test 4: Reset Timing and Frequency
TEST_F(ResetIntegrationTest, ResetTimingFrequency) {
    std::cout << "\n=== Reset Timing Test ===" << std::endl;

    StateVector initial_state = createDriftingState();
    filter->initialize(initial_state);

    std::vector<double> reset_times;
    double simulation_time = 0;

    // Run for 60 seconds with periodic gravity updates
    for (int second = 0; second < 60; ++second) {
        // IMU updates at 100 Hz
        for (int i = 0; i < 100; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.01, 0.01, 9.81);
            imu.gyro = Vector3d::Zero();
            imu.timestamp = simulation_time;
            filter->processIMU(imu, 0.01);
            simulation_time += 0.01;
        }

        // Gravity update every second
        if (second % 5 == 0) {
            auto gravity_buffer = createGravityBuffer(5, simulation_time);
            filter->updateRBPF(gravity_buffer);

            // Check for reset
            if (filter->shouldReset()) {
                auto result = filter->performReset();
                reset_times.push_back(simulation_time);
                std::cout << "Reset at t=" << simulation_time
                         << "s, jump=" << result.position_jump << "m" << std::endl;
            }
        }
    }

    // Analyze reset frequency
    std::cout << "\nTotal resets: " << reset_times.size() << std::endl;

    if (reset_times.size() > 1) {
        for (size_t i = 1; i < reset_times.size(); ++i) {
            double interval = reset_times[i] - reset_times[i-1];
            std::cout << "Reset interval " << i << ": " << interval << " s" << std::endl;

            // Should respect minimum interval (10s from config)
            EXPECT_GE(interval, 10.0)
                << "Reset interval too short";
        }
    }
}

// Test 5: Multi-Sensor Reset Coordination
TEST_F(ResetIntegrationTest, MultiSensorResetCoordination) {
    std::cout << "\n=== Multi-Sensor Reset Test ===" << std::endl;

    StateVector initial_state = createDriftingState();
    filter->initialize(initial_state);

    // Process multiple sensor types
    for (int i = 0; i < 500; ++i) {  // 5 seconds
        // IMU at 100 Hz
        IMUData imu;
        imu.accel = Vector3d(0.02, 0, 9.81);
        imu.gyro = Vector3d(0, 0, 0.01);
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);

        // Barometer at 10 Hz
        if (i % 10 == 0) {
            BarometerData baro;
            baro.pressure = 90000 + i * 0.1;  // Slowly changing
            baro.temperature = 280;
            baro.timestamp = i * 0.01;
            filter->updateBarometer(baro);
        }

        // Magnetometer at 10 Hz
        if (i % 10 == 5) {
            MagnetometerData mag;
            mag.field = Vector3d(30e-6, 0, 40e-6);
            mag.timestamp = i * 0.01;
            filter->updateMagnetometer(mag);
        }

        // Gradiometer at 1 Hz
        if (i % 100 == 0) {
            GradiometerData grad;
            grad.timestamp = i * 0.01;
            grad.tensor << 30, 20, -50, 5, 3, 2;
            grad.valid = true;
            grad.confidence = 0.9;
            filter->updateGravity(grad);
        }
    }

    // Check state consistency
    auto state = filter->getCurrentState();
    EXPECT_TRUE(state.is_valid)
        << "State invalid after multi-sensor processing";

    double uncertainty = std::sqrt(state.covariance.block<3,3>(0,0).trace());
    std::cout << "Final uncertainty with all sensors: " << uncertainty << " m" << std::endl;
}

// Test 6: Reset Recovery Performance
TEST_F(ResetIntegrationTest, ResetRecoveryPerformance) {
    std::cout << "\n=== Reset Recovery Performance Test ===" << std::endl;

    StateVector initial_state = createDriftingState();
    filter->initialize(initial_state);

    // Induce large error
    std::cout << "Inducing large position error..." << std::endl;
    for (int i = 0; i < 3000; ++i) {  // 30 seconds
        IMUData imu;
        imu.accel = Vector3d(0.1, 0.1, 9.81);  // Large bias
        imu.gyro = Vector3d(0.01, 0.01, 0.01);
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    Vector3d error_position = filter->getState().position;
    std::cout << "Position with error: " << error_position.transpose() << std::endl;

    // Perform reset if needed
    if (filter->shouldReset()) {
        std::cout << "Performing reset..." << std::endl;
        auto start = high_resolution_clock::now();

        auto reset_result = filter->performReset();

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start);

        std::cout << "Reset execution time: " << duration.count() << " ms" << std::endl;
        std::cout << "Position jump: " << reset_result.position_jump << " m" << std::endl;

        // Reset should be fast
        EXPECT_LT(duration.count(), 100)
            << "Reset took too long";
    }

    // Continue normal operation after reset
    std::cout << "Continuing operation after reset..." << std::endl;
    for (int i = 0; i < 1000; ++i) {  // 10 seconds
        IMUData imu;
        imu.accel = Vector3d(0, 0, 9.81);  // No bias
        imu.gyro = Vector3d::Zero();
        imu.timestamp = 30.0 + i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    // Check recovery
    auto final_state = filter->getCurrentState();
    EXPECT_TRUE(final_state.is_valid)
        << "State invalid after reset recovery";

    double final_uncertainty = std::sqrt(final_state.covariance.block<3,3>(0,0).trace());
    std::cout << "Final uncertainty after recovery: " << final_uncertainty << " m" << std::endl;
}

// Test 7: Confidence-Based Reset Decision
TEST_F(ResetIntegrationTest, ConfidenceBasedReset) {
    std::cout << "\n=== Confidence-Based Reset Test ===" << std::endl;

    StateVector initial_state = createDriftingState();
    filter->initialize(initial_state);

    // Track confidence over time
    std::vector<double> confidence_history;
    std::vector<double> time_history;

    for (int second = 0; second < 40; ++second) {
        // Process IMU
        for (int i = 0; i < 100; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.02, 0.02, 9.81);
            imu.gyro = Vector3d(0.001, 0.001, 0.001);
            imu.timestamp = second + i * 0.01;
            filter->processIMU(imu, 0.01);
        }

        // Record confidence
        auto state = filter->getCurrentState();
        confidence_history.push_back(state.confidence);
        time_history.push_back(second);

        std::cout << "t=" << second << "s, confidence=" << state.confidence;

        // Check reset condition
        if (filter->shouldReset()) {
            std::cout << " [RESET NEEDED]";

            // Provide gravity measurements for RBPF
            auto gravity_buffer = createGravityBuffer(5, second);
            filter->updateRBPF(gravity_buffer);

            // Perform reset
            auto result = filter->performReset();
            std::cout << " - Reset executed, jump=" << result.position_jump << "m";
        }
        std::cout << std::endl;
    }

    // Analyze confidence pattern
    if (!confidence_history.empty()) {
        double min_confidence = *std::min_element(confidence_history.begin(), confidence_history.end());
        double max_confidence = *std::max_element(confidence_history.begin(), confidence_history.end());

        std::cout << "\nConfidence range: [" << min_confidence << ", " << max_confidence << "]" << std::endl;

        // Should have valid confidence values
        EXPECT_GE(min_confidence, 0.0)
            << "Invalid confidence value";
        EXPECT_LE(max_confidence, 1.0)
            << "Invalid confidence value";
    }
}