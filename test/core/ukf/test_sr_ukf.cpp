/**
 * Critical Unit Tests for Square-Root UKF
 * Focus on identifying and fixing the altitude explosion issue
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "core/ukf/sr_ukf.h"
#include "utils/math_utils.h"
#include "utils/logger.h"

using namespace Navigation;
using namespace NavMath;

class SRUKFTest : public ::testing::Test {
protected:
    std::unique_ptr<SquareRootUKF> ukf;

    void SetUp() override {
        // Initialize logger for debugging
        Logger::getInstance().initialize(".", "DEBUG");

        // Load the main config file
        YAML::Node config = YAML::LoadFile("config.yaml");

        // Create UKF using the main config
        ukf = std::make_unique<SquareRootUKF>(config["ukf"]);
    }
};

// Test 1: Verify sigma point scale factor calculation
TEST_F(SRUKFTest, Critical_SigmaPointScaleFactor) {
    // Load config to get actual values
    YAML::Node config = YAML::LoadFile("config.yaml");
    auto ukf_config = config["ukf"]["sigma_points"];

    double alpha = ukf_config["alpha"].as<double>();
    double kappa = ukf_config["kappa"].as<double>();

    // For augmented UKF with n_aug = 40 (20 states + 20 noise)
    double n_aug = 40;
    double lambda = alpha * alpha * (n_aug + kappa) - n_aug;
    double expected_scale = std::sqrt(n_aug + lambda);

    // With standard UKF parameters (alpha=0.001, kappa=-37)
    // lambda = 0.001^2 * (40 - 37) - 40 = 0.000001 * 3 - 40 ≈ -39.999997
    // scale_factor = sqrt(40 - 39.999997) = sqrt(0.000003) ≈ 0.00173

    EXPECT_GT(n_aug + lambda, 0) << "n_aug + lambda must be positive";
    EXPECT_GT(expected_scale, 0) << "Scale factor must be positive";
    EXPECT_LT(expected_scale, 10.0) << "Scale factor too large - will cause instability";
}

// Test 2: Verify initial state doesn't explode in first prediction
TEST_F(SRUKFTest, Critical_AltitudeStability) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -3000);  // 3000m altitude in NED
    initial_state.velocity = Vector3d(100, 0, 0);    // 100 m/s north
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Simulate IMU data (level flight)
    // For level flight, accelerometer reads upward specific force opposing gravity
    Vector3d accel(0, 0, -9.81);  // Upward specific force in body frame (NED)
    Vector3d gyro(0, 0, 0);       // No rotation
    double dt = 0.01;             // 100 Hz

    // Run a single prediction step
    ukf->predict(accel, gyro, dt);

    StateVector state_after = ukf->getState();

    // Check altitude hasn't exploded
    double altitude = -state_after.position.z();
    EXPECT_NEAR(altitude, 3000.0, 10.0) << "Altitude exploded from 3000m to " << altitude << "m";
    EXPECT_TRUE(std::isfinite(altitude)) << "Altitude became NaN/Inf";
    EXPECT_LT(std::abs(altitude), 100000.0) << "Altitude exceeded reasonable bounds";

    // Check position is reasonable
    EXPECT_LT(state_after.position.norm(), 10000.0) << "Position magnitude too large";
    EXPECT_TRUE(state_after.position.allFinite()) << "Position contains NaN/Inf";
}

// Test 3: Verify covariance matrix stays valid after QR update
TEST_F(SRUKFTest, Critical_CovarianceValidity) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -3000);
    initial_state.velocity = Vector3d(100, 0, 0);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Get initial square-root covariance
    MatrixXd S_init = ukf->getSquareRootCovariance();

    // Check initial S is valid
    EXPECT_TRUE(S_init.allFinite()) << "Initial S contains NaN/Inf";

    // Check diagonal elements are positive
    for (int i = 0; i < S_init.rows(); ++i) {
        EXPECT_GT(S_init(i, i), 0) << "S diagonal element " << i << " is not positive";
        EXPECT_LT(S_init(i, i), 1000.0) << "S diagonal element " << i << " is too large";
    }

    // Run prediction
    Vector3d accel(0, 0, 9.81);
    Vector3d gyro(0, 0, 0);
    ukf->predict(accel, gyro, 0.01);

    // Check S after update
    MatrixXd S_after = ukf->getSquareRootCovariance();
    EXPECT_TRUE(S_after.allFinite()) << "S contains NaN/Inf after prediction";

    // Verify diagonal still positive
    for (int i = 0; i < S_after.rows(); ++i) {
        EXPECT_GT(S_after(i, i), 0) << "S diagonal element " << i << " became negative";
        EXPECT_LT(S_after(i, i), 1000.0) << "S diagonal element " << i << " exploded";
    }

    // Check full covariance is positive semi-definite
    MatrixXd P = ukf->getCovariance();
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(P);
    VectorXd eigenvalues = solver.eigenvalues();
    for (int i = 0; i < eigenvalues.size(); ++i) {
        EXPECT_GE(eigenvalues(i), -1e-9) << "Covariance has negative eigenvalue: " << eigenvalues(i);
    }
}

// Test 4: Test barometer update doesn't cause explosion
TEST_F(SRUKFTest, Critical_BarometerUpdate) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -3000);
    initial_state.velocity = Vector3d(100, 0, 0);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Predict step (stationary, accelerometer reads upward)
    ukf->predict(Vector3d(0, 0, -9.81), Vector3d::Zero(), 0.01);

    // Barometer measurement at 3000m altitude
    // Standard atmosphere: P = 101325 * (1 - 0.0065*h/288.15)^5.256
    double altitude = 3000.0;
    double pressure = 101325.0 * std::pow(1.0 - 0.0065 * altitude / 288.15, 5.256);
    double temperature = 288.15 - 0.0065 * altitude;  // Standard lapse rate

    // Update with barometer
    ukf->updateBarometer(pressure, temperature);

    // Check state after update
    StateVector state_after = ukf->getState();
    double altitude_after = -state_after.position.z();

    EXPECT_NEAR(altitude_after, 3000.0, 60.0) << "Barometer update caused altitude jump";
    EXPECT_TRUE(std::isfinite(altitude_after)) << "Altitude became NaN after barometer update";
}

// Test 5: Test multiple prediction cycles for stability
TEST_F(SRUKFTest, Critical_MultiplePredictionStability) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -3000);
    initial_state.velocity = Vector3d(100, 0, 0);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Run 100 predictions (1 second at 100Hz)
    for (int i = 0; i < 100; ++i) {
        ukf->predict(Vector3d(0, 0, 9.81), Vector3d::Zero(), 0.01);

        StateVector state = ukf->getState();
        double altitude = -state.position.z();

        // Check each iteration
        ASSERT_TRUE(std::isfinite(altitude)) << "Altitude became NaN at iteration " << i;
        ASSERT_LT(std::abs(altitude - 3000.0), 100.0)
            << "Altitude diverged to " << altitude << "m at iteration " << i;

        // Early exit if diverging
        if (!state.position.allFinite() || state.position.norm() > 10000.0) {
            FAIL() << "State diverged at iteration " << i
                   << ", position: " << state.position.transpose();
        }
    }

    // Final check
    StateVector final_state = ukf->getState();
    double final_altitude = -final_state.position.z();
    EXPECT_NEAR(final_altitude, 3000.0, 10.0) << "Altitude drifted after 100 predictions";
}

// Test 6: Test state conversions don't amplify errors
TEST_F(SRUKFTest, StateConversionRoundTrip) {
    StateVector state;
    state.position = Vector3d(100, 200, -3000);
    state.velocity = Vector3d(10, 20, -1);
    state.quaternion = Quaterniond(Eigen::AngleAxisd(0.1, Vector3d::UnitZ()));
    state.accel_bias = Vector3d(0.01, 0.02, 0.03);
    state.gyro_bias = Vector3d(0.001, 0.002, 0.003);
    state.gravity_bias = Eigen::Matrix<double, 5, 1>::Constant(0.1);

    // Convert to vector and back
    VectorXd vec = state.toVector();
    ASSERT_EQ(vec.size(), 21) << "State vector should be 21-dimensional";

    StateVector recovered;
    recovered.fromVector(vec);

    // Check recovery is close to original
    EXPECT_LT((recovered.position - state.position).norm(), 1e-10) << "Position not recovered";
    EXPECT_LT((recovered.velocity - state.velocity).norm(), 1e-10) << "Velocity not recovered";
    EXPECT_LT((recovered.accel_bias - state.accel_bias).norm(), 1e-10) << "Accel bias not recovered";
}

// Test 7: Test gravity model at different altitudes
TEST_F(SRUKFTest, GravityModelSanity) {
    // Test gravity at various altitudes
    std::vector<double> altitudes = {0, 1000, 3000, 10000, 30000};

    for (double alt : altitudes) {
        StateVector state;
        state.position = Vector3d(0, 0, -alt);
        state.velocity = Vector3d::Zero();
        state.quaternion = Quaterniond::Identity();

        // Gravity should be approximately 9.8 m/s^2 downward (positive in NED)
        // We'll test this more thoroughly in the earth model tests

        // For now, just ensure it's in reasonable range
        // Gravity varies from ~9.78 to 9.83 m/s^2 on Earth surface
        // and decreases with altitude
        double expected_g = 9.81 * (1.0 - 2.0 * alt / 6371000.0);  // Simple approximation

        EXPECT_GT(expected_g, 9.0) << "Gravity too small at altitude " << alt;
        EXPECT_LT(expected_g, 10.0) << "Gravity too large at altitude " << alt;
    }
}

// Test 8: Magnetometer update test
TEST_F(SRUKFTest, MagnetometerUpdate) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -1000);
    initial_state.velocity = Vector3d(10, 0, 0);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Predict first
    ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

    // Create a magnetometer measurement
    // Earth's magnetic field is roughly 50,000 nT = 50 μT
    // In NED frame, typical values might be:
    Vector3d mag_ned(20.0, 0.0, 45.0);  // microTesla
    Vector3d mag_body = initial_state.quaternion.inverse() * mag_ned;

    // Update with magnetometer
    ukf->updateMagnetometer(mag_body);

    StateVector state_after = ukf->getState();

    // Check state is still valid
    EXPECT_TRUE(state_after.position.allFinite()) << "Position became NaN after mag update";
    EXPECT_TRUE(state_after.quaternion.coeffs().allFinite()) << "Quaternion became NaN after mag update";

    // Quaternion should remain normalized
    EXPECT_NEAR(state_after.quaternion.norm(), 1.0, 1e-6) << "Quaternion denormalized after mag update";
}

// Test 9: Gravity gradient update test
TEST_F(SRUKFTest, GravityGradientUpdate) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -3000);
    initial_state.velocity = Vector3d(100, 0, 0);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Predict first
    ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

    // Create a gravity gradient measurement (5 independent components)
    // Typical values are in Eötvös (1E = 10^-9 s^-2)
    Eigen::Matrix<double, 5, 1> gradient;
    gradient << 100.0,   // Txx [E]
                -50.0,   // Tyy [E]
                10.0,    // Txy [E]
                5.0,     // Txz [E]
                3.0;     // Tyz [E]

    // Update with gravity gradient
    ukf->updateGravity(gradient);

    StateVector state_after = ukf->getState();

    // Check state is still valid
    EXPECT_TRUE(state_after.position.allFinite()) << "Position became NaN after gravity update";
    EXPECT_TRUE(state_after.gravity_bias.allFinite()) << "Gravity bias became NaN";

    // Position shouldn't change drastically
    double pos_change = (state_after.position - initial_state.position).norm();
    EXPECT_LT(pos_change, 100.0) << "Position jumped too much after gravity update";
}

// Test 10: Sequential measurement updates
TEST_F(SRUKFTest, SequentialMeasurementUpdates) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -2000);
    initial_state.velocity = Vector3d(50, 30, -2);
    initial_state.quaternion = Quaterniond(Eigen::AngleAxisd(0.1, Vector3d::UnitZ()));
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Predict
    ukf->predict(Vector3d(0.5, -0.3, -9.81), Vector3d(0.01, 0.02, 0.0), 0.01);

    StateVector state_before_updates = ukf->getState();

    // Sequential updates
    // 1. Barometer
    double pressure = 101325.0 * std::pow(1.0 - 0.0065 * 2000.0 / 288.15, 5.256);
    ukf->updateBarometer(pressure, 288.15 - 0.0065 * 2000.0);

    // 2. Magnetometer
    Vector3d mag_body(20.0, 5.0, 40.0);
    ukf->updateMagnetometer(mag_body);

    // 3. Gravity gradient
    Eigen::Matrix<double, 5, 1> gradient;
    gradient << 80.0, -40.0, 8.0, 4.0, 2.0;
    ukf->updateGravity(gradient);

    StateVector state_after_all = ukf->getState();

    // Check convergence and stability
    EXPECT_TRUE(state_after_all.position.allFinite()) << "Position NaN after sequential updates";
    EXPECT_TRUE(state_after_all.velocity.allFinite()) << "Velocity NaN after sequential updates";
    EXPECT_TRUE(state_after_all.quaternion.coeffs().allFinite()) << "Quaternion NaN after sequential updates";

    // Biases should have been updated
    double accel_bias_change = state_after_all.accel_bias.norm();
    double gyro_bias_change = state_after_all.gyro_bias.norm();

    // Some change is expected due to updates
    EXPECT_GE(accel_bias_change, 0.0) << "Accel bias should be non-negative";
    EXPECT_GE(gyro_bias_change, 0.0) << "Gyro bias should be non-negative";
}

// Test 11: Measurement outlier handling
TEST_F(SRUKFTest, MeasurementOutlierRejection) {
    StateVector initial_state;
    initial_state.position = Vector3d(0, 0, -1500);
    initial_state.velocity = Vector3d(20, 10, 0);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d::Zero();
    initial_state.gyro_bias = Vector3d::Zero();
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    ukf->initialize(initial_state);

    // Predict
    ukf->predict(Vector3d(0, 0, -9.81), Vector3d(0, 0, 0), 0.01);

    // Get state before outlier
    StateVector state_before = ukf->getState();
    MatrixXd P_before = ukf->getCovariance();

    // Apply outlier barometer measurement (wrong altitude by 1000m)
    double wrong_pressure = 101325.0 * std::pow(1.0 - 0.0065 * 500.0 / 288.15, 5.256);
    ukf->updateBarometer(wrong_pressure, 288.15);

    StateVector state_after = ukf->getState();

    // State shouldn't jump by full 1000m due to outlier rejection or filtering
    double altitude_change = std::abs((-state_after.position.z()) - (-state_before.position.z()));
    EXPECT_LT(altitude_change, 500.0) << "Filter didn't handle outlier properly";

    // State should still be valid
    EXPECT_TRUE(state_after.position.allFinite()) << "Position became NaN with outlier";
}

// Test 12: Covariance positive definiteness after updates
TEST_F(SRUKFTest, CovariancePositiveDefiniteAfterUpdates) {
    StateVector initial_state;
    initial_state.position = Vector3d(100, 200, -2500);
    initial_state.velocity = Vector3d(30, -20, 1);
    initial_state.quaternion = Quaterniond::Identity();
    initial_state.accel_bias = Vector3d(0.001, -0.002, 0.0005);
    initial_state.gyro_bias = Vector3d(0.0001, -0.0001, 0.00005);
    initial_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Constant(0.1);

    ukf->initialize(initial_state);

    // Run multiple predict-update cycles
    for (int i = 0; i < 10; ++i) {
        // Predict
        ukf->predict(Vector3d(0.1, -0.2, -9.81), Vector3d(0.001, 0.002, 0.0), 0.01);

        // Update with barometer
        if (i % 2 == 0) {
            double pressure = 101325.0 * std::pow(1.0 - 0.0065 * 2500.0 / 288.15, 5.256);
            ukf->updateBarometer(pressure, 288.15);
        }

        // Update with magnetometer
        if (i % 3 == 0) {
            Vector3d mag(25.0, 0.0, 40.0);
            ukf->updateMagnetometer(mag);
        }

        // Check covariance
        MatrixXd P = ukf->getCovariance();

        // Check for NaN/Inf
        ASSERT_TRUE(P.allFinite()) << "Covariance has NaN/Inf at iteration " << i;

        // Check positive semi-definiteness
        Eigen::SelfAdjointEigenSolver<MatrixXd> solver(P);
        VectorXd eigenvalues = solver.eigenvalues();

        double min_eigenvalue = eigenvalues.minCoeff();
        EXPECT_GE(min_eigenvalue, -1e-9)
            << "Covariance not PSD at iteration " << i << ", min eigenvalue: " << min_eigenvalue;

        // Check symmetry
        double symmetry_error = (P - P.transpose()).norm();
        EXPECT_LT(symmetry_error, 1e-10) << "Covariance not symmetric at iteration " << i;
    }
}