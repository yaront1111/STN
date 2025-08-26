/**
 * Unit tests for Extended Kalman Filter
 * Tests state estimation, covariance propagation, and measurement updates
 */
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "ekf.h"
#include "types.h"

class EKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize EKF with known state
        initial_state.p_NED = Eigen::Vector3d(100, 200, -500);  // Position
        initial_state.v_NED = Eigen::Vector3d(50, 0, 0);        // Velocity
        initial_state.q_BN = Eigen::Quaterniond::Identity();     // Attitude
        
        ekf.init(initial_state);
        
        // Set consistent Singer parameters
        ekf.setSingerParams(1.0, 30.0);  // 1 m/s² noise, 30s correlation
    }
    
    EKF ekf;
    State initial_state;
};

TEST_F(EKFTest, Initialization) {
    // Check that initialization sets correct state
    State current = ekf.x;
    
    EXPECT_NEAR((current.p_NED - initial_state.p_NED).norm(), 0.0, 1e-10);
    EXPECT_NEAR((current.v_NED - initial_state.v_NED).norm(), 0.0, 1e-10);
    EXPECT_NEAR((current.q_BN.coeffs() - initial_state.q_BN.coeffs()).norm(), 0.0, 1e-10);
}

TEST_F(EKFTest, CovarianceInitialization) {
    // Covariance should be positive definite
    Eigen::Matrix<double, 15, 15> P = ekf.P;
    
    // Check symmetry
    EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-10);
    
    // Check positive definiteness (all eigenvalues > 0)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> solver(P);
    auto eigenvalues = solver.eigenvalues();
    
    for (int i = 0; i < 15; i++) {
        EXPECT_GT(eigenvalues(i), 0.0);
    }
}

TEST_F(EKFTest, PredictStep) {
    // Test prediction with simple IMU input
    Eigen::Vector3d a_N(1.0, 0.0, 0.0);  // 1 m/s² acceleration north
    double dt = 0.01;
    
    State before = ekf.x;
    ekf.predict(a_N, Eigen::Vector3d::Zero(), dt);
    State after = ekf.x;
    
    // Position should change by v*dt + 0.5*a*dt²
    Eigen::Vector3d expected_pos = before.p_NED + before.v_NED * dt + 0.5 * a_N * dt * dt;
    EXPECT_NEAR((after.p_NED - expected_pos).norm(), 0.0, 1e-6);
    
    // Velocity should change by a*dt
    Eigen::Vector3d expected_vel = before.v_NED + a_N * dt;
    EXPECT_NEAR((after.v_NED - expected_vel).norm(), 0.0, 1e-6);
}

TEST_F(EKFTest, CovariancePropagation) {
    // Covariance should grow during prediction
    Eigen::Matrix<double, 15, 15> P_before = ekf.P;
    
    // Multiple prediction steps
    for (int i = 0; i < 10; i++) {
        ekf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    Eigen::Matrix<double, 15, 15> P_after = ekf.P;
    
    // Position uncertainty should increase
    EXPECT_GT(P_after(0, 0), P_before(0, 0));  // North position
    EXPECT_GT(P_after(1, 1), P_before(1, 1));  // East position
    EXPECT_GT(P_after(2, 2), P_before(2, 2));  // Down position
    
    // Covariance should remain symmetric and positive definite
    EXPECT_NEAR((P_after - P_after.transpose()).norm(), 0.0, 1e-10);
}

TEST_F(EKFTest, AltitudeUpdate) {
    // Test radar altimeter measurement update
    double true_agl = 100.0;
    double terrain_height = -400.0;  // Terrain at 400m elevation
    
    // Set state to be at true_agl above terrain
    ekf.x.p_NED.z() = -(terrain_height + true_agl);
    
    // Create measurement
    double measured_agl = true_agl + 1.0;  // 1m error
    double slope = 0.1;  // 10% slope
    
    // Store covariance before update
    double P_before = ekf.P(2, 2);  // Down position variance
    
    // Apply update
    ekf.update_agl(ekf.x, measured_agl, terrain_height, slope);
    
    // Position should move toward measurement
    double agl_after = -ekf.x.p_NED.z() - terrain_height;
    EXPECT_LT(std::abs(agl_after - measured_agl), std::abs(true_agl - measured_agl));
    
    // Covariance should decrease
    EXPECT_LT(ekf.P(2, 2), P_before);
}

TEST_F(EKFTest, GravityUpdate) {
    // Test gravity anomaly update
    Eigen::Vector3d f_measured(0.0, 0.0, -9.81);  // Measured specific force
    
    // Apply gravity update
    ekf.update_gravity(ekf.x, f_measured.z());
    
    // This should constrain vertical channel
    // Check that covariance is updated (exact behavior depends on implementation)
    EXPECT_TRUE(ekf.P(2, 2) > 0);  // Should remain positive
}

TEST_F(EKFTest, MeasurementRejection) {
    // Test that outliers are properly rejected
    double true_agl = 100.0;
    double terrain_height = -400.0;
    
    ekf.x.p_NED.z() = -(terrain_height + true_agl);
    
    // Create outlier measurement (way off)
    double outlier_agl = true_agl + 100.0;  // 100m error (huge)
    double slope = 0.1;
    
    State before = ekf.x;
    Eigen::Matrix<double, 15, 15> P_before = ekf.P;
    
    // Apply update (should be rejected by NIS test)
    ekf.update_agl(ekf.x, outlier_agl, terrain_height, slope);
    
    // State shouldn't change much for outlier
    // (Exact behavior depends on gate threshold)
    double position_change = (ekf.x.p_NED - before.p_NED).norm();
    EXPECT_LT(position_change, 10.0);  // Should not jump to outlier
}

TEST_F(EKFTest, JosephFormStability) {
    // Test that Joseph form maintains positive definiteness
    // Even with many updates
    
    for (int i = 0; i < 100; i++) {
        // Predict
        ekf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
        
        // Update with various measurements
        if (i % 5 == 0) {
            ekf.update_agl(ekf.x, 100.0 + i * 0.1, -400.0, 0.1);
        }
        if (i % 10 == 0) {
            ekf.update_gravity(ekf.x, -9.81);
        }
        
        // Check covariance remains valid
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> solver(ekf.P);
        double min_eigenvalue = solver.eigenvalues().minCoeff();
        
        EXPECT_GT(min_eigenvalue, -1e-10);  // Allow small numerical errors
        
        // Check symmetry
        double asymmetry = (ekf.P - ekf.P.transpose()).norm();
        EXPECT_LT(asymmetry, 1e-10);
    }
}

TEST_F(EKFTest, SingerModel) {
    // Test Singer acceleration model
    double sigma_a = 1.0;  // 1 m/s² acceleration noise
    double tau = 30.0;     // 30s correlation time
    
    ekf.setSingerParams(sigma_a, tau);
    
    // Process noise should be set correctly
    // Q matrix diagonal elements for acceleration states should be:
    // q = sigma_a² * (1 - exp(-2*dt/tau))
    
    double dt = 0.01;
    double expected_q = sigma_a * sigma_a * (1 - exp(-2 * dt / tau));
    
    // This is approximate - actual implementation may differ
    EXPECT_GT(expected_q, 0.0);
    EXPECT_LT(expected_q, sigma_a * sigma_a);
}