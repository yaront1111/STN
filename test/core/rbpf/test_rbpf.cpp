/**
 * Unit Tests for Rao-Blackwellized Particle Filter
 * Tests particle filter with analytical sub-filters
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/rbpf/rbpf.h"
#include "core/ukf/sr_ukf.h"

using namespace Eigen;
using namespace Navigation;

class RBPFTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::RBPF> rbpf;
    Navigation::RBPFConfig config;

    void SetUp() override {
        // Configure RBPF for testing
        config.num_particles = 100;  // Fewer particles for faster tests
        config.effective_sample_size_threshold = 0.5;
        config.position_noise_std = 1.0;
        config.altitude_noise_std = 0.5;
        config.deterministic = true;  // For reproducible tests
        config.random_seed = 42;
        config.parallel_evaluation = false;  // Simplify debugging

        rbpf = std::make_unique<Navigation::RBPF>(config);
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
TEST_F(RBPFTest, Initialization) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 0.1;

    rbpf->initialize(initial_state, initial_cov);

    // Get estimate and check it's close to initial state
    StateVector estimated = rbpf->getMMSE();

    EXPECT_LT((estimated.position - initial_state.position).norm(), 0.1)
        << "Position initialization incorrect";
    EXPECT_LT((estimated.velocity - initial_state.velocity).norm(), 0.1)
        << "Velocity initialization incorrect";

    // Check effective sample size
    double ess = rbpf->getEffectiveSampleSize();
    EXPECT_GT(ess, config.num_particles * 0.9)
        << "ESS too low after initialization";
}

// Test prediction step
TEST_F(RBPFTest, Prediction) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16));

    // Apply constant acceleration
    Vector3d accel(0, 0, 9.81);  // Gravity
    Vector3d gyro(0, 0, 0);       // No rotation
    double dt = 0.1;

    StateVector before = rbpf->getMMSE();
    rbpf->predict(accel, gyro, dt);
    StateVector after = rbpf->getMMSE();

    // Position should change based on velocity
    Vector3d expected_pos = before.position + before.velocity * dt;
    EXPECT_LT((after.position - expected_pos).norm(), 1.0)
        << "Position prediction incorrect";

    // Velocity should change based on acceleration
    Vector3d expected_vel = before.velocity + accel * dt;
    EXPECT_LT((after.velocity - expected_vel).norm(), 1.0)
        << "Velocity prediction incorrect";
}

// Test gravity update
TEST_F(RBPFTest, GravityUpdate) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16) * 10.0);

    // Create gravity measurement (5-element STF vector)
    Eigen::Matrix<double, 5, 1> gravity_meas;
    gravity_meas << 0.1, 0.2, -0.3, 0.05, 0.02;  // Example STF values

    rbpf->updateGravity(gravity_meas);

    // After update, particle weights should have changed
    auto weights = rbpf->getParticleWeights();

    // Check that weights are not all equal (indicates update worked)
    double weight_variance = 0;
    double mean_weight = 1.0 / weights.size();
    for (double w : weights) {
        weight_variance += (w - mean_weight) * (w - mean_weight);
    }

    EXPECT_GT(weight_variance, 1e-10)
        << "Weights not updated by gravity measurement";
}

// Test terrain update
TEST_F(RBPFTest, TerrainUpdate) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16));

    double terrain_altitude = -950.0;  // 950m altitude
    rbpf->updateTerrain(terrain_altitude);

    // Get updated estimate
    StateVector updated = rbpf->getMMSE();

    // Altitude should be influenced by terrain measurement
    EXPECT_NEAR(updated.position.z(), terrain_altitude, 100.0)
        << "Terrain update didn't affect altitude estimate";
}

// Test barometer update
TEST_F(RBPFTest, BarometerUpdate) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16));

    // Sea level pressure adjusted for altitude
    double pressure = 90000.0;     // Lower pressure at altitude
    double temperature = 280.0;    // Kelvin

    rbpf->updateBarometer(pressure, temperature);

    StateVector updated = rbpf->getMMSE();

    // Barometer should refine altitude estimate
    EXPECT_LT(std::abs(updated.position.z() + 1000), 200.0)
        << "Barometer update didn't improve altitude";
}

// Test resampling trigger
TEST_F(RBPFTest, ResamplingTrigger) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16));

    // Check if resampling is needed initially
    bool needs_resample = rbpf->needsResampling();
    EXPECT_FALSE(needs_resample)
        << "Shouldn't need resampling right after initialization";

    // Apply many updates to degrade particle weights
    Eigen::Matrix<double, 5, 1> gravity;
    gravity << 1.0, 1.0, 1.0, 1.0, 1.0;  // Outlier measurement

    for (int i = 0; i < 10; ++i) {
        rbpf->updateGravity(gravity);
    }

    // Now should need resampling due to weight degeneracy
    double ess = rbpf->getEffectiveSampleSize();
    EXPECT_LT(ess, config.num_particles * config.effective_sample_size_threshold)
        << "ESS didn't decrease with outlier measurements";
}

// Test particle position diversity
TEST_F(RBPFTest, ParticleDiversity) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16);
    initial_cov.block<3, 3>(0, 0) *= 100.0;  // Large position uncertainty

    rbpf->initialize(initial_state, initial_cov);

    auto positions = rbpf->getParticlePositions();

    // Check that particles are spread out
    Vector3d mean_pos = Vector3d::Zero();
    for (const auto& pos : positions) {
        mean_pos += pos;
    }
    mean_pos /= positions.size();

    double variance = 0;
    for (const auto& pos : positions) {
        variance += (pos - mean_pos).squaredNorm();
    }
    variance /= positions.size();

    EXPECT_GT(variance, 10.0)
        << "Particles not sufficiently diverse";
}

// Test MAP vs MMSE estimates
TEST_F(RBPFTest, MAPvsMMSE) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16) * 10.0);

    // Add some measurements to create non-uniform weights
    Vector3d accel(0.1, 0.2, 9.81);
    Vector3d gyro(0.01, 0.02, 0.03);
    rbpf->predict(accel, gyro, 0.1);

    StateVector mmse = rbpf->getMMSE();
    StateVector map = rbpf->getMAP();

    // MAP and MMSE should be different with non-uniform weights
    double position_diff = (map.position - mmse.position).norm();

    // They should be close but not identical
    EXPECT_GT(position_diff, 0.001)
        << "MAP and MMSE identical (should differ)";
    EXPECT_LT(position_diff, 100.0)
        << "MAP and MMSE too different";
}

// Test covariance estimation
TEST_F(RBPFTest, CovarianceEstimation) {
    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16) * 5.0;

    rbpf->initialize(initial_state, initial_cov);

    MatrixXd cov = rbpf->getCovariance();

    // Check covariance dimensions
    EXPECT_EQ(cov.rows(), 16)
        << "Covariance has wrong number of rows";
    EXPECT_EQ(cov.cols(), 16)
        << "Covariance has wrong number of columns";

    // Check positive definiteness
    Eigen::SelfAdjointEigenSolver<MatrixXd> es(cov);
    double min_eigenvalue = es.eigenvalues().minCoeff();
    EXPECT_GT(min_eigenvalue, 0)
        << "Covariance not positive definite";

    // Check symmetry
    EXPECT_LT((cov - cov.transpose()).norm(), 1e-10)
        << "Covariance not symmetric";
}

// Test position hypothesis injection
TEST_F(RBPFTest, PositionHypothesisInjection) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16));

    // Inject a new position hypothesis
    Vector3d new_position(1000, 2000, -500);
    double confidence = 0.8;

    rbpf->injectPositionHypothesis(new_position, confidence);

    StateVector updated = rbpf->getMMSE();

    // Position should move toward injected hypothesis
    double dist_to_new = (updated.position - new_position).norm();
    double dist_to_old = (updated.position - initial_state.position).norm();

    EXPECT_LT(dist_to_new, dist_to_old)
        << "Position didn't move toward injected hypothesis";
}

// Test filter statistics
TEST_F(RBPFTest, FilterStatistics) {
    StateVector initial_state = createInitialState();
    rbpf->initialize(initial_state, MatrixXd::Identity(16, 16));

    // Perform some operations
    Vector3d accel(0, 0, 9.81);
    Vector3d gyro(0, 0, 0.1);
    rbpf->predict(accel, gyro, 0.01);

    auto stats = rbpf->getStatistics();

    // Check statistics are reasonable
    EXPECT_GT(stats.effective_sample_size, 0)
        << "Invalid effective sample size";
    EXPECT_LE(stats.effective_sample_size, config.num_particles)
        << "ESS exceeds particle count";

    EXPECT_GE(stats.max_weight, 1.0 / config.num_particles)
        << "Max weight too small";
    EXPECT_LE(stats.max_weight, 1.0)
        << "Max weight exceeds 1";

    EXPECT_GE(stats.weight_variance, 0)
        << "Negative weight variance";
}

// Test deterministic mode
TEST_F(RBPFTest, DeterministicMode) {
    // Create two filters with same config
    RBPFConfig det_config = config;
    det_config.deterministic = true;
    det_config.random_seed = 123;

    auto rbpf1 = std::make_unique<RBPF>(det_config);
    auto rbpf2 = std::make_unique<RBPF>(det_config);

    StateVector initial_state = createInitialState();
    MatrixXd initial_cov = MatrixXd::Identity(16, 16);

    rbpf1->initialize(initial_state, initial_cov);
    rbpf2->initialize(initial_state, initial_cov);

    // Apply same updates
    Vector3d accel(1, 2, 9.81);
    Vector3d gyro(0.1, 0.2, 0.3);

    rbpf1->predict(accel, gyro, 0.1);
    rbpf2->predict(accel, gyro, 0.1);

    // Results should be identical
    StateVector state1 = rbpf1->getMMSE();
    StateVector state2 = rbpf2->getMMSE();

    EXPECT_LT((state1.position - state2.position).norm(), 1e-10)
        << "Deterministic mode not reproducible";
}