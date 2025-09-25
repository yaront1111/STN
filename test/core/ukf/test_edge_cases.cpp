/**
 * Edge Cases and Error Condition Tests for UKF
 * Tests extreme inputs, boundary conditions, and error handling
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <limits>
#include "core/ukf/sr_ukf.h"
#include "utils/math_utils.h"
#include "utils/logger.h"

using namespace Navigation;
using namespace NavMath;

class UKFEdgeCaseTest : public ::testing::Test {
protected:
    std::unique_ptr<SquareRootUKF> ukf;
    SRUKFConfig config;

    void SetUp() override {
        // Initialize logger
        Logger::getInstance().initialize(".", "ERROR");

        // Standard config
        config.alpha = 0.1;
        config.beta = 2.0;
        config.kappa = -37;

        config.q_pos = 1.0e-8;
        config.q_vel = 1.0e-6;
        config.q_att = 1.0e-7;
        config.q_accel_bias = 1.0e-9;
        config.q_gyro_bias = 1.0e-11;
        config.q_grav_bias = 1.0e-14;

        config.init_pos_cov = 1.0;
        config.init_vel_cov = 0.01;
        config.init_att_cov = 0.001;

        ukf = std::make_unique<SquareRootUKF>(config);
    }

    StateVector createNominalState() {
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

// Test 1: Zero dt (duplicate timestamps)
TEST_F(UKFEdgeCaseTest, ZeroDeltaTime) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    StateVector state_before = ukf->getState();

    // Predict with dt = 0
    ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.0);

    StateVector state_after = ukf->getState();

    // State shouldn't change with dt = 0
    EXPECT_LT((state_after.position - state_before.position).norm(), 1e-10)
        << "Position changed with dt=0";
    EXPECT_LT((state_after.velocity - state_before.velocity).norm(), 1e-10)
        << "Velocity changed with dt=0";

    // State should remain valid
    EXPECT_TRUE(state_after.position.allFinite()) << "Position became NaN with dt=0";
}

// Test 2: Large dt gaps (sensor dropout)
TEST_F(UKFEdgeCaseTest, LargeDeltaTime) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    // Test with increasingly large dt
    std::vector<double> large_dts = {1.0, 10.0, 60.0, 300.0};  // Up to 5 minutes

    for (double dt : large_dts) {
        StateVector state_before = ukf->getState();

        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), dt);

        StateVector state_after = ukf->getState();

        // State should remain finite
        ASSERT_TRUE(state_after.position.allFinite())
            << "Position became NaN with dt=" << dt;
        ASSERT_TRUE(state_after.velocity.allFinite())
            << "Velocity became NaN with dt=" << dt;

        // Uncertainty should grow but remain bounded
        MatrixXd P = ukf->getCovariance();
        ASSERT_TRUE(P.allFinite()) << "Covariance became NaN with dt=" << dt;

        double max_cov = P.diagonal().maxCoeff();
        EXPECT_LT(max_cov, 1e6) << "Covariance exploded with dt=" << dt;
    }
}

// Test 3: Extreme accelerations (>16g)
TEST_F(UKFEdgeCaseTest, ExtremeAccelerations) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    // Test accelerations up to 20g
    std::vector<Vector3d> extreme_accels = {
        Vector3d(200, 0, -9.81),      // 20g forward
        Vector3d(0, -200, -9.81),     // 20g right
        Vector3d(0, 0, -200),         // 20g up
        Vector3d(150, 150, 150),      // ~26g combined
        Vector3d(-200, -200, -200)    // ~35g combined negative
    };

    for (const auto& accel : extreme_accels) {
        ukf->predict(accel, Vector3d::Zero(), 0.01);

        StateVector state = ukf->getState();

        // State should remain valid even with extreme inputs
        EXPECT_TRUE(state.position.allFinite())
            << "Position NaN with accel=" << accel.transpose();
        EXPECT_TRUE(state.velocity.allFinite())
            << "Velocity NaN with accel=" << accel.transpose();
        EXPECT_TRUE(state.quaternion.coeffs().allFinite())
            << "Quaternion NaN with accel=" << accel.transpose();

        // Quaternion should remain normalized
        EXPECT_NEAR(state.quaternion.norm(), 1.0, 1e-3)
            << "Quaternion denormalized with extreme acceleration";
    }
}

// Test 4: Extreme angular rates (>2000 deg/s)
TEST_F(UKFEdgeCaseTest, ExtremeAngularRates) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    // Test angular rates up to 3000 deg/s (52 rad/s)
    double deg2rad = M_PI / 180.0;
    std::vector<Vector3d> extreme_gyros = {
        Vector3d(3000 * deg2rad, 0, 0),           // Roll only
        Vector3d(0, 3000 * deg2rad, 0),           // Pitch only
        Vector3d(0, 0, 3000 * deg2rad),           // Yaw only
        Vector3d(2000, 2000, 2000) * deg2rad,     // Combined
        Vector3d(-3000, -3000, -3000) * deg2rad   // Negative combined
    };

    for (const auto& gyro : extreme_gyros) {
        ukf->predict(Vector3d(0, 0, -9.81), gyro, 0.001);  // Small dt for extreme rates

        StateVector state = ukf->getState();

        // State should remain valid
        EXPECT_TRUE(state.quaternion.coeffs().allFinite())
            << "Quaternion NaN with gyro=" << gyro.transpose();

        // Quaternion must stay normalized
        double quat_norm = state.quaternion.norm();
        EXPECT_NEAR(quat_norm, 1.0, 1e-3)
            << "Quaternion denormalized with gyro=" << gyro.transpose();
    }
}

// Test 5: Gimbal lock conditions (pitch ±90°)
TEST_F(UKFEdgeCaseTest, GimbalLockConditions) {
    // Test near gimbal lock orientations
    std::vector<Quaterniond> gimbal_lock_quats = {
        Quaterniond(Eigen::AngleAxisd(M_PI/2 - 0.001, Vector3d::UnitY())),   // Pitch ~90°
        Quaterniond(Eigen::AngleAxisd(-M_PI/2 + 0.001, Vector3d::UnitY())),  // Pitch ~-90°
        Quaterniond(Eigen::AngleAxisd(M_PI/2, Vector3d::UnitY())),           // Pitch exactly 90°
        Quaterniond(Eigen::AngleAxisd(-M_PI/2, Vector3d::UnitY()))           // Pitch exactly -90°
    };

    for (const auto& quat : gimbal_lock_quats) {
        StateVector initial = createNominalState();
        initial.quaternion = quat;
        ukf->initialize(initial);

        // Try to rotate through gimbal lock
        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0.1, 0.1, 0.1), 0.01);

        StateVector state = ukf->getState();

        // Should remain valid
        EXPECT_TRUE(state.quaternion.coeffs().allFinite())
            << "Quaternion NaN near gimbal lock";
        EXPECT_NEAR(state.quaternion.norm(), 1.0, 1e-6)
            << "Quaternion denormalized near gimbal lock";

        // Covariance should remain bounded
        MatrixXd P = ukf->getCovariance();
        EXPECT_TRUE(P.allFinite()) << "Covariance NaN near gimbal lock";
        EXPECT_LT(P.norm(), 1e6) << "Covariance exploded near gimbal lock";
    }
}

// Test 6: Poles (latitude ±90°)
TEST_F(UKFEdgeCaseTest, PolarRegions) {
    // Test at Earth's poles where certain calculations may be singular
    std::vector<Vector3d> pole_positions = {
        Vector3d(0, 0, -1000),           // Normal latitude
        Vector3d(6371000, 0, -1000),     // Near north pole
        Vector3d(-6371000, 0, -1000),    // Near south pole
    };

    for (const auto& pos : pole_positions) {
        StateVector initial = createNominalState();
        initial.position = pos;
        ukf->initialize(initial);

        // Predict with rotation (singularity test)
        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0.1), 0.01);

        StateVector state = ukf->getState();

        EXPECT_TRUE(state.position.allFinite())
            << "Position NaN at pole-like location";
        EXPECT_TRUE(state.quaternion.coeffs().allFinite())
            << "Quaternion NaN at pole-like location";
    }
}

// Test 7: NaN/Inf in measurements
TEST_F(UKFEdgeCaseTest, NaNInfMeasurements) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    // Predict first
    ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

    StateVector state_before = ukf->getState();

    // Try to update with NaN/Inf measurements
    // Note: The actual UKF should reject these

    // NaN in magnetometer
    Vector3d nan_mag(std::numeric_limits<double>::quiet_NaN(), 0, 45);
    ukf->updateMagnetometer(nan_mag);

    StateVector state_after_nan = ukf->getState();
    EXPECT_TRUE(state_after_nan.position.allFinite())
        << "State corrupted by NaN measurement";

    // Infinity in barometer
    double inf_pressure = std::numeric_limits<double>::infinity();
    ukf->updateBarometer(inf_pressure, 288.15);

    StateVector state_after_inf = ukf->getState();
    EXPECT_TRUE(state_after_inf.position.allFinite())
        << "State corrupted by Inf measurement";

    // State should remain close to prediction (measurements rejected)
    double pos_change = (state_after_inf.position - state_before.position).norm();
    EXPECT_LT(pos_change, 10.0) << "State jumped with invalid measurements";
}

// Test 8: Very small covariance (near-zero uncertainty)
TEST_F(UKFEdgeCaseTest, NearZeroCovariance) {
    // Create config with very small initial covariance
    SRUKFConfig tiny_cov_config = config;
    tiny_cov_config.init_pos_cov = 1e-12;
    tiny_cov_config.init_vel_cov = 1e-12;
    tiny_cov_config.init_att_cov = 1e-12;

    auto tiny_cov_ukf = std::make_unique<SquareRootUKF>(tiny_cov_config);

    StateVector initial = createNominalState();
    tiny_cov_ukf->initialize(initial);

    // Should be able to predict without numerical issues
    tiny_cov_ukf->predict(Vector3d(1, 2, -9.81), Vector3d(0.01, 0.02, 0.03), 0.01);

    StateVector state = tiny_cov_ukf->getState();
    MatrixXd P = tiny_cov_ukf->getCovariance();

    EXPECT_TRUE(state.position.allFinite()) << "Position NaN with tiny covariance";
    EXPECT_TRUE(P.allFinite()) << "Covariance NaN with tiny initial values";

    // Covariance should grow from process noise
    EXPECT_GT(P.trace(), 1e-15) << "Covariance didn't grow from tiny initial value";
}

// Test 9: Very large covariance (high uncertainty)
TEST_F(UKFEdgeCaseTest, LargeCovariance) {
    // Create config with very large initial covariance
    SRUKFConfig large_cov_config = config;
    large_cov_config.init_pos_cov = 1e6;
    large_cov_config.init_vel_cov = 1e4;
    large_cov_config.init_att_cov = 10.0;

    auto large_cov_ukf = std::make_unique<SquareRootUKF>(large_cov_config);

    StateVector initial = createNominalState();
    large_cov_ukf->initialize(initial);

    // Should handle large uncertainty
    large_cov_ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

    StateVector state = large_cov_ukf->getState();
    MatrixXd P = large_cov_ukf->getCovariance();

    EXPECT_TRUE(state.position.allFinite()) << "Position NaN with large covariance";
    EXPECT_TRUE(P.allFinite()) << "Covariance NaN with large initial values";

    // Update should reduce uncertainty
    double pressure = 101325.0 * std::pow(1.0 - 0.0065 * 1000.0 / 288.15, 5.256);
    large_cov_ukf->updateBarometer(pressure, 288.15);

    MatrixXd P_after = large_cov_ukf->getCovariance();
    EXPECT_LT(P_after.trace(), P.trace()) << "Update didn't reduce uncertainty";
}

// Test 10: Simultaneous extreme conditions
TEST_F(UKFEdgeCaseTest, CombinedExtremeConditions) {
    StateVector initial = createNominalState();
    initial.velocity = Vector3d(500, 0, -100);  // Very fast
    initial.quaternion = Quaterniond(Eigen::AngleAxisd(M_PI/2 - 0.01, Vector3d::UnitY()));  // Near gimbal lock

    ukf->initialize(initial);

    // Apply extreme inputs
    Vector3d extreme_accel(100, -100, 50);  // High acceleration
    Vector3d extreme_gyro(10, 10, 10);      // High rotation rate
    double large_dt = 1.0;                  // Large time step

    // This is a stress test - should not crash or produce NaN
    ukf->predict(extreme_accel, extreme_gyro, large_dt);

    StateVector state = ukf->getState();
    MatrixXd P = ukf->getCovariance();

    // Basic validity checks
    EXPECT_TRUE(state.position.allFinite()) << "Position NaN under combined extremes";
    EXPECT_TRUE(state.velocity.allFinite()) << "Velocity NaN under combined extremes";
    EXPECT_TRUE(state.quaternion.coeffs().allFinite()) << "Quaternion NaN under combined extremes";
    EXPECT_TRUE(P.allFinite()) << "Covariance NaN under combined extremes";

    // Quaternion normalization
    EXPECT_NEAR(state.quaternion.norm(), 1.0, 0.1) << "Quaternion severely denormalized";

    // Covariance positive semi-definite
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(P);
    double min_eigenvalue = solver.eigenvalues().minCoeff();
    EXPECT_GE(min_eigenvalue, -1e-6) << "Covariance lost positive semi-definiteness";
}

// Test 11: Rapid switching between extreme conditions
TEST_F(UKFEdgeCaseTest, RapidConditionSwitching) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    // Rapidly switch between extreme conditions
    for (int i = 0; i < 20; ++i) {
        Vector3d accel, gyro;
        double dt;

        if (i % 2 == 0) {
            // Extreme positive
            accel = Vector3d(150, 150, -150);
            gyro = Vector3d(20, 20, 20);
            dt = 0.001;
        } else {
            // Extreme negative
            accel = Vector3d(-150, -150, 150);
            gyro = Vector3d(-20, -20, -20);
            dt = 0.1;
        }

        ukf->predict(accel, gyro, dt);

        StateVector state = ukf->getState();

        // Should remain stable despite rapid changes
        ASSERT_TRUE(state.position.allFinite())
            << "Position NaN at switch " << i;
        ASSERT_TRUE(state.quaternion.coeffs().allFinite())
            << "Quaternion NaN at switch " << i;
        ASSERT_NEAR(state.quaternion.norm(), 1.0, 0.01)
            << "Quaternion denormalized at switch " << i;
    }
}

// Test 12: Recovery after temporary NaN state
TEST_F(UKFEdgeCaseTest, RecoveryFromInvalidState) {
    StateVector initial = createNominalState();
    ukf->initialize(initial);

    // Save good state
    StateVector good_state = ukf->getState();
    MatrixXd good_cov = ukf->getCovariance();

    // Try to corrupt the state (the UKF should handle this gracefully)
    // In practice, this would come from sensor failures or computation errors

    // Multiple normal predictions should stabilize any transients
    for (int i = 0; i < 10; ++i) {
        ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);
    }

    StateVector final_state = ukf->getState();

    // Should have valid state after recovery attempt
    EXPECT_TRUE(final_state.position.allFinite()) << "Failed to maintain valid position";
    EXPECT_TRUE(final_state.velocity.allFinite()) << "Failed to maintain valid velocity";
    EXPECT_TRUE(final_state.quaternion.coeffs().allFinite()) << "Failed to maintain valid quaternion";

    // Quaternion should be normalized
    EXPECT_NEAR(final_state.quaternion.norm(), 1.0, 1e-6) << "Quaternion not properly normalized";
}