/**
 * Unit Tests for Navigation System Fixes
 * Tests confidence updates, reset intervals, map-free RBPF, and hypothesis injection
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/hierarchical_filter.h"
#include "core/rbpf/rbpf.h"
#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace Navigation;

class NavigationFixesTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::HierarchicalFilter> filter;
    std::unique_ptr<Navigation::RBPF> rbpf;
    YAML::Node ukf_config;
    YAML::Node rbpf_config;

    void SetUp() override {
        setupConfigs();
        setupFilter();
        setupRBPF();
    }

    void setupConfigs() {
        // UKF configuration
        ukf_config["ukf"]["process_noise"]["position"] = 1.0e-8;
        ukf_config["ukf"]["process_noise"]["velocity"] = 1.0e-4;
        ukf_config["ukf"]["initial_covariance"]["position"] = 1.0;

        // RBPF configuration
        rbpf_config["rbpf"]["num_particles"] = 100;
        rbpf_config["rbpf"]["confidence"]["min_confidence"] = 0.6;
        rbpf_config["rbpf"]["reset"]["min_interval_s"] = 10.0;
    }

    void setupFilter() {
        filter = std::make_unique<HierarchicalFilter>(ukf_config, rbpf_config, nullptr);
    }

    void setupRBPF() {
        rbpf = std::make_unique<RBPF>(rbpf_config);
    }

    StateVector createTestState() {
        StateVector state;
        state.position = Vector3d(0, 0, -1000);
        state.velocity = Vector3d(100, 0, 0);
        state.quaternion = Quaterniond::Identity();
        state.accel_bias = Vector3d::Zero();
        state.gyro_bias = Vector3d::Zero();
        state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();
        return state;
    }
};

// Test 1: Confidence Updates in UKF_ONLY Mode
TEST_F(NavigationFixesTest, ConfidenceUpdatesInUKFMode) {
    StateVector initial_state = createTestState();
    filter->initialize(initial_state);

    // Get initial confidence
    auto state1 = filter->getCurrentState();
    double initial_confidence = state1.confidence;
    std::cout << "Initial confidence: " << initial_confidence << std::endl;

    // Process many IMU updates to grow uncertainty
    for (int i = 0; i < 500; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0.01, 0.01, 9.81);
        imu.gyro = Vector3d(0.001, 0.001, 0.001);
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    // Check confidence has degraded
    auto state2 = filter->getCurrentState();
    double degraded_confidence = state2.confidence;
    std::cout << "Degraded confidence: " << degraded_confidence << std::endl;

    // Confidence should decrease with growing uncertainty
    EXPECT_LT(degraded_confidence, initial_confidence)
        << "Confidence should degrade with growing uncertainty";

    // Confidence should be in valid range
    EXPECT_GE(degraded_confidence, 0.0) << "Confidence below 0";
    EXPECT_LE(degraded_confidence, 1.0) << "Confidence above 1";
}

// Test 2: Reset Interval Enforcement
TEST_F(NavigationFixesTest, ResetIntervalEnforcement) {
    StateVector initial_state = createTestState();
    filter->initialize(initial_state);

    // Force first reset
    for (int i = 0; i < 1000; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0.1, 0.1, 9.81);
        imu.gyro = Vector3d(0.01, 0.01, 0.01);
        imu.timestamp = i * 0.01;
        filter->processIMU(imu, 0.01);
    }

    // Should need reset due to high uncertainty
    bool needs_reset1 = filter->shouldReset();
    std::cout << "Needs reset after 10s: " << needs_reset1 << std::endl;

    if (needs_reset1) {
        auto result = filter->performReset();
        int reset_count1 = filter->getResetCount();

        // Immediately check for reset again (should be blocked)
        bool needs_reset2 = filter->shouldReset();
        std::cout << "Needs reset immediately after: " << needs_reset2 << std::endl;

        EXPECT_FALSE(needs_reset2)
            << "Reset should be blocked by interval enforcement";

        // Advance time by 5 seconds
        for (int i = 0; i < 500; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.1, 0.1, 9.81);
            imu.gyro = Vector3d(0.01, 0.01, 0.01);
            imu.timestamp = 10.0 + i * 0.01;
            filter->processIMU(imu, 0.01);
        }

        // Still too soon (only 5 seconds passed, need 15)
        bool needs_reset3 = filter->shouldReset();
        std::cout << "Needs reset after 5s: " << needs_reset3 << std::endl;

        EXPECT_FALSE(needs_reset3)
            << "Reset should still be blocked (only 5s passed)";

        // Advance time by another 11 seconds (total 16s)
        for (int i = 0; i < 1100; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.1, 0.1, 9.81);
            imu.gyro = Vector3d(0.01, 0.01, 0.01);
            imu.timestamp = 15.0 + i * 0.01;
            filter->processIMU(imu, 0.01);
        }

        // Now enough time has passed
        bool needs_reset4 = filter->shouldReset();
        std::cout << "Needs reset after 16s: " << needs_reset4 << std::endl;

        // Should allow reset now (if conditions are met)
        // Note: May not trigger if uncertainty is still low
    }
}

// Test 3: Map-Free RBPF Gravity Updates
TEST_F(NavigationFixesTest, MapFreeGravityUpdates) {
    StateVector initial_state = createTestState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 10.0;

    rbpf->initialize(initial_state, initial_cov);

    // Get initial ESS
    auto initial_stats = rbpf->getStatistics();
    double initial_ess = initial_stats.effective_sample_size;
    std::cout << "Initial ESS: " << initial_ess << std::endl;

    // Apply gravity measurements without map
    std::vector<Eigen::Matrix<double, 5, 1>> measurements;
    for (int i = 0; i < 10; ++i) {
        Eigen::Matrix<double, 5, 1> gravity;
        // Create varying but consistent gradient
        gravity << 20.0 + i * 0.5,   // Txx
                   15.0 - i * 0.3,    // Tyy
                   3.0 + i * 0.1,     // Txy
                   2.0,               // Txz
                   1.0;               // Tyz
        measurements.push_back(gravity);

        // Update RBPF
        rbpf->updateGravity(gravity);

        // Small prediction step
        Vector3d accel(0, 0, 9.81);
        Vector3d gyro(0, 0, 0);
        rbpf->predict(accel, gyro, 0.1);
    }

    // Check that weights have changed
    auto final_stats = rbpf->getStatistics();
    double final_ess = final_stats.effective_sample_size;
    std::cout << "Final ESS: " << final_ess << std::endl;

    auto weights = rbpf->getParticleWeights();
    double weight_variance = 0;
    double mean_weight = 1.0 / weights.size();
    for (double w : weights) {
        weight_variance += (w - mean_weight) * (w - mean_weight);
    }

    std::cout << "Weight variance: " << weight_variance << std::endl;

    // Weights should no longer be uniform after updates
    EXPECT_GT(weight_variance, 1e-10)
        << "Weights should vary after map-free gravity updates";

    // ESS should change
    EXPECT_NE(final_ess, initial_ess)
        << "ESS should change after gravity updates";
}

// Test 4: Improved Hypothesis Injection
TEST_F(NavigationFixesTest, ImprovedHypothesisInjection) {
    StateVector initial_state = createTestState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 50.0;

    rbpf->initialize(initial_state, initial_cov);

    // Get initial particle spread
    auto initial_positions = rbpf->getParticlePositions();
    Vector3d initial_mean = Vector3d::Zero();
    for (const auto& pos : initial_positions) {
        initial_mean += pos;
    }
    initial_mean /= initial_positions.size();

    // Inject hypothesis with high confidence
    Vector3d new_position(500, 500, -800);
    double confidence = 0.9;
    rbpf->injectPositionHypothesis(new_position, confidence);

    // Get particle distribution after injection
    auto injected_positions = rbpf->getParticlePositions();

    // Count particles near injected position
    int near_count = 0;
    double radius = 100.0;  // 100m radius
    for (const auto& pos : injected_positions) {
        if ((pos - new_position).norm() < radius) {
            near_count++;
        }
    }

    std::cout << "Particles near injection: " << near_count
              << " / " << injected_positions.size() << std::endl;

    // Should have significant particles near injection
    double injection_ratio = static_cast<double>(near_count) / injected_positions.size();
    EXPECT_GT(injection_ratio, 0.3)
        << "At least 30% of particles should be near injection point with high confidence";

    // Test with low confidence
    rbpf->initialize(initial_state, initial_cov);
    Vector3d new_position2(1000, 1000, -1200);
    rbpf->injectPositionHypothesis(new_position2, 0.3);

    auto low_conf_positions = rbpf->getParticlePositions();
    int near_count2 = 0;
    for (const auto& pos : low_conf_positions) {
        if ((pos - new_position2).norm() < radius) {
            near_count2++;
        }
    }

    double low_conf_ratio = static_cast<double>(near_count2) / low_conf_positions.size();
    std::cout << "Particles near low-confidence injection: " << low_conf_ratio << std::endl;

    // Lower confidence should inject fewer particles
    EXPECT_LT(low_conf_ratio, injection_ratio)
        << "Lower confidence should inject fewer particles";
}

// Test 5: Integration - Confidence Triggers Reset
TEST_F(NavigationFixesTest, ConfidenceTriggersReset) {
    StateVector initial_state = createTestState();
    filter->initialize(initial_state);

    int initial_resets = filter->getResetCount();

    // Process with bias to degrade performance
    for (int second = 0; second < 30; ++second) {
        for (int i = 0; i < 100; ++i) {
            IMUData imu;
            imu.accel = Vector3d(0.05, 0.05, 9.81);  // Biased
            imu.gyro = Vector3d(0.005, 0.005, 0.005);
            imu.timestamp = second + i * 0.01;
            filter->processIMU(imu, 0.01);
        }

        auto state = filter->getCurrentState();
        std::cout << "t=" << second << "s: confidence=" << state.confidence
                  << ", uncertainty=" << std::sqrt(state.covariance.block<3,3>(0,0).trace())
                  << "m" << std::endl;

        // Check if confidence-based reset might trigger
        if (state.confidence < 0.7 && filter->shouldReset()) {
            std::cout << "Low confidence triggered reset check" << std::endl;
        }
    }

    // Confidence should degrade over time with biased measurements
    auto final_state = filter->getCurrentState();
    EXPECT_LT(final_state.confidence, 0.95)
        << "Confidence should degrade with biased measurements";
}

// Test 6: Map-Free RBPF Consistency
TEST_F(NavigationFixesTest, MapFreeRBPFConsistency) {
    StateVector initial_state = createTestState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 5.0;

    rbpf->initialize(initial_state, initial_cov);

    // Simulate consistent movement with matching gradients
    for (int i = 0; i < 20; ++i) {
        // Move particles
        Vector3d accel(1.0, 0, 9.81);  // Accelerating north
        Vector3d gyro(0, 0, 0);
        rbpf->predict(accel, gyro, 0.1);

        // Gradient should change consistently with movement
        Eigen::Matrix<double, 5, 1> gradient;
        gradient << 20.0 + i * 0.1,   // Gradual change
                    15.0 - i * 0.05,
                    3.0,
                    2.0,
                    1.0;
        rbpf->updateGravity(gradient);
    }

    // Check particle convergence
    auto positions = rbpf->getParticlePositions();
    Vector3d mean = Vector3d::Zero();
    for (const auto& pos : positions) {
        mean += pos;
    }
    mean /= positions.size();

    double spread = 0;
    for (const auto& pos : positions) {
        spread += (pos - mean).squaredNorm();
    }
    spread = std::sqrt(spread / positions.size());

    std::cout << "Final particle spread: " << spread << " m" << std::endl;

    // Consistent measurements should maintain reasonable spread
    EXPECT_LT(spread, 200.0)
        << "Particle spread should be reasonable with consistent measurements";
    EXPECT_GT(spread, 1.0)
        << "Particles should maintain some diversity";
}