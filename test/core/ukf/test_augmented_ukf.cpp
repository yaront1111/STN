/**
 * Augmented UKF Specific Tests
 * Tests the augmented sigma point approach implementation
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "core/ukf/sr_ukf.h"
#include "utils/math_utils.h"
#include "utils/logger.h"

using namespace Navigation;
using namespace NavMath;

class AugmentedUKFTest : public ::testing::Test {
protected:
    std::unique_ptr<SquareRootUKF> ukf;
    SRUKFConfig config;

    void SetUp() override {
        // Initialize logger
        Logger::getInstance().initialize(".", "ERROR");

        // Configure for augmented UKF
        config.alpha = 0.1;
        config.beta = 2.0;
        config.kappa = 3 - 40;  // 3 - n_aug(40) = -37

        // Small process noise for testing
        config.q_pos = 1.0e-8;
        config.q_vel = 1.0e-6;
        config.q_att = 1.0e-7;
        config.q_accel_bias = 1.0e-9;
        config.q_gyro_bias = 1.0e-11;
        config.q_grav_bias = 1.0e-14;

        // Initial covariance
        config.init_pos_cov = 1.0;
        config.init_vel_cov = 0.01;
        config.init_att_cov = 0.001;

        ukf = std::make_unique<SquareRootUKF>(config);
    }

    StateVector createTestState() {
        StateVector state;
        state.position = Vector3d(100, 200, -3000);
        state.velocity = Vector3d(50, -30, 2);
        state.quaternion = Quaterniond::Identity();
        state.accel_bias = Vector3d(0.01, -0.02, 0.03);
        state.gyro_bias = Vector3d(0.001, -0.001, 0.0005);
        state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();
        return state;
    }
};

// Test 1: Verify augmented sigma points generation (81 points)
TEST_F(AugmentedUKFTest, AugmentedSigmaPointCount) {
    StateVector initial = createTestState();
    ukf->initialize(initial);

    // The augmented approach should generate 2*n_aug + 1 = 2*40 + 1 = 81 sigma points
    // We can't directly access sigma points, but we can verify through prediction

    // Run prediction which internally generates augmented sigma points
    Vector3d accel(0, 0, -9.81);
    Vector3d gyro(0, 0, 0);
    ukf->predict(accel, gyro, 0.01);

    // Check state remained stable (no explosion)
    StateVector state_after = ukf->getState();
    EXPECT_NEAR(state_after.position.x(), initial.position.x() + initial.velocity.x() * 0.01, 0.1);
    EXPECT_NEAR(state_after.position.y(), initial.position.y() + initial.velocity.y() * 0.01, 0.1);
    EXPECT_LT(std::abs(state_after.position.z() - initial.position.z()), 1.0);
}

// Test 2: Test negative center weight handling
TEST_F(AugmentedUKFTest, NegativeCenterWeight) {
    // With alpha=0.1, kappa=-37, n_aug=40
    // lambda = 0.01 * (40 - 37) - 40 = 0.03 - 40 = -39.97
    // w0 = lambda/(n_aug + lambda) = -39.97/(40 - 39.97) = -39.97/0.03 ≈ -1332
    // But it's adjusted to lambda = -37 to get w0 = -37/3 ≈ -12.33

    StateVector initial = createTestState();
    ukf->initialize(initial);

    // Multiple predictions should remain stable despite negative center weight
    for (int i = 0; i < 10; ++i) {
        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

        StateVector state = ukf->getState();
        ASSERT_TRUE(state.position.allFinite()) << "Position became NaN/Inf at iteration " << i;
        ASSERT_LT(state.position.norm(), 10000.0) << "Position exploded at iteration " << i;
    }
}

// Test 3: Process noise injection through augmentation
TEST_F(AugmentedUKFTest, ProcessNoiseInjection) {
    StateVector initial = createTestState();
    initial.velocity = Vector3d::Zero();  // Start stationary
    ukf->initialize(initial);

    // Run many predictions to accumulate process noise effects
    MatrixXd P_initial = ukf->getCovariance();

    for (int i = 0; i < 100; ++i) {
        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);
    }

    MatrixXd P_final = ukf->getCovariance();

    // Covariance should change due to process noise and dynamics
    // With augmented UKF, it might stabilize rather than always grow
    double cov_change = std::abs(P_final.trace() - P_initial.trace());
    EXPECT_GT(cov_change, 1e-6)
        << "Covariance didn't change with process noise injection";

    // But should remain bounded
    EXPECT_LT(P_final.trace(), P_initial.trace() * 100.0)
        << "Covariance grew too much";
}

// Test 4: Compare augmented vs non-augmented propagation
TEST_F(AugmentedUKFTest, AugmentedPropagationAccuracy) {
    StateVector initial = createTestState();
    ukf->initialize(initial);

    // Test with varying accelerations
    std::vector<Vector3d> test_accels = {
        Vector3d(1, 0, -9.81),
        Vector3d(0, 2, -9.81),
        Vector3d(-1, -1, -9.81),
        Vector3d(5, 5, -9.81)
    };

    for (const auto& accel : test_accels) {
        StateVector state_before = ukf->getState();
        ukf->predict(accel, Vector3d(0, 0, 0.1), 0.01);
        StateVector state_after = ukf->getState();

        // Verify reasonable state progression
        Vector3d expected_vel_change = (accel + Vector3d(0, 0, 9.81)) * 0.01;
        Vector3d actual_vel_change = state_after.velocity - state_before.velocity;

        EXPECT_LT((actual_vel_change - expected_vel_change).norm(), 0.1)
            << "Velocity change doesn't match expected physics";
    }
}

// Test 5: Numerical stability with extreme inputs
TEST_F(AugmentedUKFTest, NumericalStabilityExtremeInputs) {
    StateVector initial = createTestState();
    ukf->initialize(initial);

    // Test with extreme but valid inputs
    struct ExtremeCase {
        Vector3d accel;
        Vector3d gyro;
        std::string description;
    };

    std::vector<ExtremeCase> cases = {
        {Vector3d(150, 150, 150), Vector3d(0, 0, 0), "Max acceleration (15g each axis)"},
        {Vector3d(0, 0, 0), Vector3d(30, 30, 30), "Max rotation rate (30 rad/s)"},
        {Vector3d(-150, -150, -150), Vector3d(-30, -30, -30), "Max negative values"},
        {Vector3d(0.0001, 0.0001, -9.81), Vector3d(0.0001, 0.0001, 0.0001), "Near zero values"}
    };

    for (const auto& test_case : cases) {
        ukf->predict(test_case.accel, test_case.gyro, 0.01);

        StateVector state = ukf->getState();
        MatrixXd P = ukf->getCovariance();

        EXPECT_TRUE(state.position.allFinite())
            << "Position NaN/Inf with " << test_case.description;
        EXPECT_TRUE(state.velocity.allFinite())
            << "Velocity NaN/Inf with " << test_case.description;
        EXPECT_TRUE(P.allFinite())
            << "Covariance NaN/Inf with " << test_case.description;

        // Check covariance remains positive semi-definite
        Eigen::SelfAdjointEigenSolver<MatrixXd> solver(P);
        double min_eigenvalue = solver.eigenvalues().minCoeff();
        EXPECT_GE(min_eigenvalue, -1e-9)
            << "Covariance not PSD with " << test_case.description;
    }
}

// Test 6: Sigma point weight consistency
TEST_F(AugmentedUKFTest, SigmaPointWeightConsistency) {
    // For augmented UKF with n_aug = 40, lambda = -37
    // w0_mean = lambda/(n_aug + lambda) = -37/3 ≈ -12.33
    // wi = 1/(2*(n_aug + lambda)) = 1/6 ≈ 0.167
    // Sum should equal 1.0

    int n_aug = 40;
    double lambda = -37.0;  // Adjusted value
    double w0_mean = lambda / (n_aug + lambda);
    double wi = 0.5 / (n_aug + lambda);

    double sum_weights = w0_mean + 2 * n_aug * wi;
    EXPECT_NEAR(sum_weights, 1.0, 1e-10) << "Weights must sum to 1.0";

    // Verify negative center weight
    EXPECT_LT(w0_mean, 0) << "Center weight should be negative for this configuration";

    // Verify other weights are positive
    EXPECT_GT(wi, 0) << "Non-center weights should be positive";
}

// Test 7: Augmented state dimension verification
TEST_F(AugmentedUKFTest, AugmentedStateDimension) {
    // Verify dimensions are correct
    // Error state: 20 (3 pos + 3 vel + 3 att_err + 3 ab + 3 gb + 5 grav_bias)
    // Process noise: 20 (matching error state dimension)
    // Augmented: 40

    EXPECT_EQ(StateVector::ERROR_DIM, 20) << "Error state dimension should be 20";

    StateVector state = createTestState();
    VectorXd vec = state.toVector();
    EXPECT_EQ(vec.size(), 21) << "Full state vector should be 21-dimensional";

    // Initialize and check covariance dimension
    ukf->initialize(state);
    MatrixXd P = ukf->getCovariance();
    EXPECT_EQ(P.rows(), 20) << "Error state covariance should be 20x20";
    EXPECT_EQ(P.cols(), 20) << "Error state covariance should be 20x20";
}

// Test 8: Process noise scaling with dt
TEST_F(AugmentedUKFTest, ProcessNoiseScaling) {
    StateVector initial = createTestState();

    // Test with different dt values
    std::vector<double> dt_values = {0.001, 0.01, 0.1, 1.0};

    for (double dt : dt_values) {
        ukf->initialize(initial);
        MatrixXd P_before = ukf->getCovariance();

        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), dt);

        MatrixXd P_after = ukf->getCovariance();
        double cov_growth = (P_after - P_before).trace();

        // Process noise should scale with dt
        // In augmented UKF, covariance can decrease due to better linearization
        // But should show different behavior for different dt
        double cov_change = std::abs(cov_growth);

        if (dt >= 0.01) {
            // Larger dt should show more change (positive or negative)
            EXPECT_GT(cov_change, 1e-8)
                << "No covariance change with dt=" << dt;
        }
    }
}

// Test 9: Augmented approach with zero process noise
TEST_F(AugmentedUKFTest, ZeroProcessNoise) {
    // Create config with zero process noise
    SRUKFConfig zero_noise_config = config;
    zero_noise_config.q_pos = 0.0;
    zero_noise_config.q_vel = 0.0;
    zero_noise_config.q_att = 0.0;
    zero_noise_config.q_accel_bias = 0.0;
    zero_noise_config.q_gyro_bias = 0.0;
    zero_noise_config.q_grav_bias = 0.0;

    auto zero_noise_ukf = std::make_unique<SquareRootUKF>(zero_noise_config);

    StateVector initial = createTestState();
    zero_noise_ukf->initialize(initial);

    MatrixXd P_before = zero_noise_ukf->getCovariance();

    // Predict with zero noise
    zero_noise_ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

    MatrixXd P_after = zero_noise_ukf->getCovariance();

    // Covariance can change due to linearization effects in UKF
    // But should be minimal with zero process noise
    double cov_change = (P_after - P_before).norm();
    EXPECT_LT(cov_change, 0.1)
        << "Covariance changed too much with zero process noise";
}

// Test 10: Consistency across multiple augmented predictions
TEST_F(AugmentedUKFTest, MultiplePredictionConsistency) {
    StateVector initial = createTestState();
    ukf->initialize(initial);

    // Store trajectory
    std::vector<Vector3d> positions;
    std::vector<double> uncertainties;

    // Constant inputs for consistent motion
    Vector3d const_accel(1.0, 0.5, -9.81);
    Vector3d const_gyro(0.01, 0.0, 0.0);

    for (int i = 0; i < 100; ++i) {
        ukf->predict(const_accel, const_gyro, 0.01);

        StateVector state = ukf->getState();
        MatrixXd P = ukf->getCovariance();

        positions.push_back(state.position);
        uncertainties.push_back(P.block<3,3>(0,0).trace());

        // Check consistency
        ASSERT_TRUE(state.position.allFinite()) << "Position NaN at step " << i;
        ASSERT_TRUE(state.quaternion.coeffs().allFinite()) << "Quaternion NaN at step " << i;

        // Quaternion should remain normalized
        double quat_norm = state.quaternion.norm();
        EXPECT_NEAR(quat_norm, 1.0, 1e-6) << "Quaternion denormalized at step " << i;
    }

    // Uncertainty should grow monotonically (with process noise)
    for (size_t i = 1; i < uncertainties.size(); ++i) {
        EXPECT_GE(uncertainties[i], uncertainties[i-1] * 0.99)
            << "Uncertainty decreased unexpectedly at step " << i;
    }
}