/**
 * Enhanced Unit Tests for RBPF
 * Focus on confidence metrics, reset injection, and gravity integration
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/rbpf/rbpf.h"
#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace Navigation;

class RBPFEnhancedTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::RBPF> rbpf;
    YAML::Node config_yaml;

    void SetUp() override {
        // Setup YAML configuration
        setupConfig();

        // Create RBPF with YAML config
        rbpf = std::make_unique<Navigation::RBPF>(config_yaml);
    }

    void setupConfig() {
        config_yaml["rbpf"]["num_particles"] = 500;
        config_yaml["rbpf"]["adaptive_particles"]["enable"] = true;
        config_yaml["rbpf"]["adaptive_particles"]["min_particles"] = 100;
        config_yaml["rbpf"]["adaptive_particles"]["max_particles"] = 1000;

        config_yaml["rbpf"]["confidence"]["calibrated"] = true;
        config_yaml["rbpf"]["confidence"]["multimodal_check"] = true;
        config_yaml["rbpf"]["confidence"]["hartigan_threshold"] = 0.05;
        config_yaml["rbpf"]["confidence"]["min_confidence"] = 0.7;
        config_yaml["rbpf"]["confidence"]["min_hold_time_s"] = 3.0;

        config_yaml["rbpf"]["reset"]["enable"] = true;
        config_yaml["rbpf"]["reset"]["require_unimodal"] = false;
        config_yaml["rbpf"]["reset"]["min_interval_s"] = 15.0;
        config_yaml["rbpf"]["reset"]["nees_upper_guard"] = 12.8;

        config_yaml["rbpf"]["resampling"]["method"] = "systematic";
        config_yaml["rbpf"]["resampling"]["threshold"] = 0.7;
        config_yaml["rbpf"]["resampling"]["roughening"]["enable"] = true;
        config_yaml["rbpf"]["resampling"]["roughening"]["adaptive"] = true;

        config_yaml["rbpf"]["proposal"] = "ukf_based";
        config_yaml["rbpf"]["update_interval_s"] = 1.0;
    }

    StateVector createInitialState() {
        StateVector state;
        state.position = Vector3d(1000, 2000, -3000);  // 3000m altitude
        state.velocity = Vector3d(100, 0, 0);          // 100 m/s north
        state.quaternion = Quaterniond::Identity();
        state.accel_bias = Vector3d(0.01, 0.02, 0.03);
        state.gyro_bias = Vector3d(0.001, 0.002, 0.003);
        state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();
        state.timestamp = 0.0;
        return state;
    }
};

// Test 1: Confidence Metric Calculation
TEST_F(RBPFEnhancedTest, ConfidenceMetricCalculation) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 10.0;

    rbpf->initialize(initial_state, initial_cov);

    // Get initial confidence (should be moderate with initial spread)
    auto stats = rbpf->getStatistics();
    double initial_ess = stats.effective_sample_size;
    std::cout << "Initial ESS: " << initial_ess << std::endl;
    std::cout << "Initial max weight: " << stats.max_weight << std::endl;
    std::cout << "Initial weight variance: " << stats.weight_variance << std::endl;

    // ESS should be close to particle count initially
    EXPECT_GT(initial_ess, 400)
        << "Initial ESS too low";

    // Apply consistent measurements to increase confidence
    for (int i = 0; i < 10; ++i) {
        Eigen::Matrix<double, 5, 1> good_gravity;
        good_gravity << 10.0, 5.0, 2.0, 1.0, 0.5;  // Consistent gradient
        rbpf->updateGravity(good_gravity);
    }

    auto good_stats = rbpf->getStatistics();
    std::cout << "\nAfter good measurements:" << std::endl;
    std::cout << "ESS: " << good_stats.effective_sample_size << std::endl;
    std::cout << "Max weight: " << good_stats.max_weight << std::endl;

    // Now apply inconsistent measurements to degrade confidence
    for (int i = 0; i < 10; ++i) {
        Eigen::Matrix<double, 5, 1> bad_gravity;
        bad_gravity = Eigen::Matrix<double, 5, 1>::Random() * 100.0;  // Random noise
        rbpf->updateGravity(bad_gravity);
    }

    auto bad_stats = rbpf->getStatistics();
    std::cout << "\nAfter bad measurements:" << std::endl;
    std::cout << "ESS: " << bad_stats.effective_sample_size << std::endl;
    std::cout << "Max weight: " << bad_stats.max_weight << std::endl;

    // ESS should degrade with bad measurements
    EXPECT_LT(bad_stats.effective_sample_size, good_stats.effective_sample_size)
        << "ESS didn't degrade with inconsistent measurements";
}

// Test 2: Particle Weight Distribution
TEST_F(RBPFEnhancedTest, ParticleWeightDistribution) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 5.0;

    rbpf->initialize(initial_state, initial_cov);

    // Check initial weight distribution
    auto weights = rbpf->getParticleWeights();
    std::cout << "Number of particles: " << weights.size() << std::endl;

    // All weights should sum to 1
    double weight_sum = 0;
    for (double w : weights) {
        weight_sum += w;
    }
    EXPECT_NEAR(weight_sum, 1.0, 1e-6)
        << "Weights don't sum to 1";

    // Initially weights should be uniform
    double expected_weight = 1.0 / weights.size();
    double max_deviation = 0;
    for (double w : weights) {
        max_deviation = std::max(max_deviation, std::abs(w - expected_weight));
    }
    std::cout << "Max deviation from uniform: " << max_deviation << std::endl;

    EXPECT_LT(max_deviation, 0.01)
        << "Initial weights not uniform";
}

// Test 3: Reset Injection with Various Confidences
TEST_F(RBPFEnhancedTest, ResetInjectionConfidenceLevels) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 20.0;

    rbpf->initialize(initial_state, initial_cov);

    // Save initial position
    auto initial_mmse = rbpf->getMMSE();
    std::cout << "Initial position: " << initial_mmse.position.transpose() << std::endl;

    // Test different confidence levels for injection
    std::vector<double> confidence_levels = {0.3, 0.5, 0.7, 0.9};
    Vector3d new_position(5000, 5000, -2000);

    for (double confidence : confidence_levels) {
        // Reset RBPF
        rbpf->initialize(initial_state, initial_cov);

        // Inject hypothesis
        std::cout << "\nInjecting with confidence: " << confidence << std::endl;
        rbpf->injectPositionHypothesis(new_position, confidence);

        // Check how much position moved
        auto updated = rbpf->getMMSE();
        double move_ratio = (updated.position - new_position).norm() /
                           (initial_state.position - new_position).norm();

        std::cout << "Position after injection: " << updated.position.transpose() << std::endl;
        std::cout << "Move ratio: " << move_ratio << std::endl;

        // Higher confidence should move position more
        if (confidence > 0.7) {
            EXPECT_LT(move_ratio, 0.5)
                << "High confidence injection didn't move position enough";
        }
    }
}

// Test 4: Gravity Measurement Weight Updates
TEST_F(RBPFEnhancedTest, GravityMeasurementWeightUpdate) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 10.0;

    rbpf->initialize(initial_state, initial_cov);

    // Spread particles by prediction with noise
    for (int i = 0; i < 5; ++i) {
        Vector3d accel(0.1, 0.1, 9.81);
        Vector3d gyro(0.01, 0.01, 0.01);
        rbpf->predict(accel, gyro, 0.1);
    }

    // Get initial weights
    auto initial_weights = rbpf->getParticleWeights();

    // Apply gravity measurement
    Eigen::Matrix<double, 5, 1> gravity;
    gravity << 15.0, 10.0, 3.0, 2.0, 1.0;  // Realistic gradient

    std::cout << "Applying gravity measurement: " << gravity.transpose() << std::endl;
    rbpf->updateGravity(gravity);

    // Get updated weights
    auto updated_weights = rbpf->getParticleWeights();

    // Calculate weight entropy (measure of distribution)
    double initial_entropy = 0;
    double updated_entropy = 0;

    for (size_t i = 0; i < initial_weights.size(); ++i) {
        if (initial_weights[i] > 1e-10) {
            initial_entropy -= initial_weights[i] * std::log(initial_weights[i]);
        }
        if (updated_weights[i] > 1e-10) {
            updated_entropy -= updated_weights[i] * std::log(updated_weights[i]);
        }
    }

    std::cout << "Initial entropy: " << initial_entropy << std::endl;
    std::cout << "Updated entropy: " << updated_entropy << std::endl;

    // Entropy should decrease (weights more concentrated)
    EXPECT_LT(updated_entropy, initial_entropy)
        << "Gravity update didn't concentrate weights";
}

// Test 5: Multimodal Distribution Detection
TEST_F(RBPFEnhancedTest, MultimodalDistributionDetection) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 100.0;  // Large uncertainty

    rbpf->initialize(initial_state, initial_cov);

    // Create bimodal distribution by injecting two hypotheses
    Vector3d hypothesis1(0, 0, -1000);
    Vector3d hypothesis2(10000, 10000, -5000);

    rbpf->injectPositionHypothesis(hypothesis1, 0.5);
    rbpf->injectPositionHypothesis(hypothesis2, 0.5);

    // Check particle positions
    auto positions = rbpf->getParticlePositions();

    // Calculate clusters
    int cluster1_count = 0;
    int cluster2_count = 0;

    for (const auto& pos : positions) {
        double dist1 = (pos - hypothesis1).norm();
        double dist2 = (pos - hypothesis2).norm();

        if (dist1 < 1000) cluster1_count++;
        if (dist2 < 1000) cluster2_count++;
    }

    std::cout << "Particles near hypothesis 1: " << cluster1_count << std::endl;
    std::cout << "Particles near hypothesis 2: " << cluster2_count << std::endl;

    // Should have particles in both clusters
    EXPECT_GT(cluster1_count, 50)
        << "Not enough particles near first hypothesis";
    EXPECT_GT(cluster2_count, 50)
        << "Not enough particles near second hypothesis";
}

// Test 6: Reset Trigger Based on Confidence
TEST_F(RBPFEnhancedTest, ResetTriggerOnLowConfidence) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 5.0;

    rbpf->initialize(initial_state, initial_cov);

    // Degrade particle distribution with many outliers
    std::cout << "\nDegrading particle distribution..." << std::endl;

    for (int i = 0; i < 20; ++i) {
        // Apply random gravity measurements (outliers)
        Eigen::Matrix<double, 5, 1> outlier_gravity;
        outlier_gravity = Eigen::Matrix<double, 5, 1>::Random() * 500.0;
        rbpf->updateGravity(outlier_gravity);
    }

    auto stats = rbpf->getStatistics();
    double ess_ratio = stats.effective_sample_size / 500.0;  // Assuming 500 particles

    std::cout << "ESS after outliers: " << stats.effective_sample_size << std::endl;
    std::cout << "ESS ratio: " << ess_ratio << std::endl;
    std::cout << "Max weight: " << stats.max_weight << std::endl;

    // Check if resampling is needed
    bool needs_resampling = rbpf->needsResampling();
    std::cout << "Needs resampling: " << needs_resampling << std::endl;

    // Should need resampling due to weight degeneracy
    EXPECT_TRUE(needs_resampling || ess_ratio < 0.5)
        << "Should need resampling after many outliers";
}

// Test 7: Particle Convergence After Reset
TEST_F(RBPFEnhancedTest, ParticleConvergenceAfterReset) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 50.0;

    rbpf->initialize(initial_state, initial_cov);

    // Check initial spread
    auto initial_positions = rbpf->getParticlePositions();
    Vector3d initial_mean = Vector3d::Zero();
    for (const auto& pos : initial_positions) {
        initial_mean += pos;
    }
    initial_mean /= initial_positions.size();

    double initial_spread = 0;
    for (const auto& pos : initial_positions) {
        initial_spread += (pos - initial_mean).squaredNorm();
    }
    initial_spread = std::sqrt(initial_spread / initial_positions.size());

    std::cout << "Initial spread: " << initial_spread << " m" << std::endl;

    // Apply strong position hypothesis (like a reset)
    Vector3d reset_position(2000, 3000, -2500);
    rbpf->injectPositionHypothesis(reset_position, 0.95);

    // Apply some consistent measurements to help convergence
    for (int i = 0; i < 5; ++i) {
        Eigen::Matrix<double, 5, 1> gravity;
        gravity << 12.0, 8.0, 3.0, 1.5, 0.8;
        rbpf->updateGravity(gravity);
    }

    // Check spread after reset
    auto reset_positions = rbpf->getParticlePositions();
    Vector3d reset_mean = Vector3d::Zero();
    for (const auto& pos : reset_positions) {
        reset_mean += pos;
    }
    reset_mean /= reset_positions.size();

    double reset_spread = 0;
    for (const auto& pos : reset_positions) {
        reset_spread += (pos - reset_mean).squaredNorm();
    }
    reset_spread = std::sqrt(reset_spread / reset_positions.size());

    std::cout << "Spread after reset: " << reset_spread << " m" << std::endl;
    std::cout << "Mean position after reset: " << reset_mean.transpose() << std::endl;

    // Particles should converge (lower spread)
    EXPECT_LT(reset_spread, initial_spread * 0.7)
        << "Particles didn't converge after reset";

    // Mean should be close to reset position
    EXPECT_LT((reset_mean - reset_position).norm(), 500)
        << "Particles didn't converge to reset position";
}

// Test 8: Gradient Matching Performance
TEST_F(RBPFEnhancedTest, GradientMatchingImprovement) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 10.0;

    rbpf->initialize(initial_state, initial_cov);

    // Define true gradient at true position
    Eigen::Matrix<double, 5, 1> true_gradient;
    true_gradient << 20.0, 15.0, 5.0, 3.0, 2.0;

    std::cout << "\nTrue gradient: " << true_gradient.transpose() << std::endl;

    // Measure initial position error
    Vector3d true_position = initial_state.position;
    auto initial_estimate = rbpf->getMMSE();
    double initial_error = (initial_estimate.position - true_position).norm();
    std::cout << "Initial position error: " << initial_error << " m" << std::endl;

    // Apply multiple consistent gradient measurements
    for (int i = 0; i < 20; ++i) {
        // Add small noise to gradient
        Eigen::Matrix<double, 5, 1> measured_gradient = true_gradient +
            Eigen::Matrix<double, 5, 1>::Random() * 1.0;
        rbpf->updateGravity(measured_gradient);

        // Small prediction step
        Vector3d accel(0, 0, 9.81);
        Vector3d gyro(0, 0, 0);
        rbpf->predict(accel, gyro, 0.05);
    }

    // Check final position error
    auto final_estimate = rbpf->getMMSE();
    double final_error = (final_estimate.position - true_position).norm();
    std::cout << "Final position error: " << final_error << " m" << std::endl;

    // Get final statistics
    auto final_stats = rbpf->getStatistics();
    std::cout << "Final ESS: " << final_stats.effective_sample_size << std::endl;

    // Position estimate should improve or at least not degrade much
    EXPECT_LT(final_error, initial_error * 1.5)
        << "Position estimate degraded despite good gradient measurements";
}

// Test 9: Adaptive Particle Count
TEST_F(RBPFEnhancedTest, AdaptiveParticleCount) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 5.0;

    rbpf->initialize(initial_state, initial_cov);

    // Get initial particle count
    auto initial_weights = rbpf->getParticleWeights();
    int initial_count = initial_weights.size();
    std::cout << "Initial particle count: " << initial_count << std::endl;

    // Create challenging scenario (high uncertainty)
    for (int i = 0; i < 10; ++i) {
        // Large prediction uncertainty
        Vector3d accel(1.0, 1.0, 9.81);
        Vector3d gyro(0.1, 0.1, 0.1);
        rbpf->predict(accel, gyro, 0.1);

        // Conflicting measurements
        Eigen::Matrix<double, 5, 1> gradient;
        gradient = Eigen::Matrix<double, 5, 1>::Random() * 50.0;
        rbpf->updateGravity(gradient);
    }

    // Check if particle count adapted (if adaptive is enabled)
    auto adapted_weights = rbpf->getParticleWeights();
    int adapted_count = adapted_weights.size();
    std::cout << "Adapted particle count: " << adapted_count << std::endl;

    // Particle count should be within configured bounds
    EXPECT_GE(adapted_count, 100)  // min_particles from config
        << "Particle count below minimum";
    EXPECT_LE(adapted_count, 1000)  // max_particles from config
        << "Particle count above maximum";
}

// Test 10: Covariance Consistency
TEST_F(RBPFEnhancedTest, CovarianceConsistency) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 10.0;

    rbpf->initialize(initial_state, initial_cov);

    // Process several updates
    for (int i = 0; i < 10; ++i) {
        Vector3d accel(0.1, 0.1, 9.81);
        Vector3d gyro(0.01, 0.01, 0.01);
        rbpf->predict(accel, gyro, 0.1);

        if (i % 2 == 0) {
            Eigen::Matrix<double, 5, 1> gravity;
            gravity << 10.0, 8.0, 3.0, 2.0, 1.0;
            rbpf->updateGravity(gravity);
        }
    }

    // Get covariance estimate
    MatrixXd cov = rbpf->getCovariance();

    // Check covariance properties
    std::cout << "\nCovariance diagonal: " << cov.diagonal().transpose() << std::endl;

    // Check positive definiteness
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(cov);
    double min_eigenvalue = solver.eigenvalues().minCoeff();
    double max_eigenvalue = solver.eigenvalues().maxCoeff();

    std::cout << "Min eigenvalue: " << min_eigenvalue << std::endl;
    std::cout << "Max eigenvalue: " << max_eigenvalue << std::endl;
    std::cout << "Condition number: " << max_eigenvalue / min_eigenvalue << std::endl;

    EXPECT_GT(min_eigenvalue, 1e-6)
        << "Covariance not positive definite";

    EXPECT_LT(max_eigenvalue / min_eigenvalue, 1e8)
        << "Covariance is ill-conditioned";

    // Check symmetry
    double symmetry_error = (cov - cov.transpose()).norm();
    EXPECT_LT(symmetry_error, 1e-10)
        << "Covariance not symmetric";
}