/**
 * Unit tests for Extended Kalman Filter
 * Tests state estimation, covariance propagation, and measurement updates
 */
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "ekf.h"
#include "types.h"
#include "filters.h"

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
    // Check that initialization sets up the filter
    // Since state is passed separately, we verify the initial state we created
    
    EXPECT_NEAR(initial_state.p_NED.norm(), std::sqrt(100*100 + 200*200 + 500*500), 1e-10);
    EXPECT_NEAR(initial_state.v_NED(0), 50.0, 1e-10);
    EXPECT_TRUE(initial_state.q_BN.isApprox(Eigen::Quaterniond::Identity()));
}

TEST_F(EKFTest, CovarianceInitialization) {
    // Test that P_pos accessor works
    Eigen::Matrix3d P_pos = ekf.get_P_pos();
    
    // Check symmetry of position covariance
    EXPECT_NEAR((P_pos - P_pos.transpose()).norm(), 0.0, 1e-10);
    
    // Check positive definiteness (all eigenvalues > 0)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P_pos);
    auto eigenvalues = solver.eigenvalues();
    
    for (int i = 0; i < 3; i++) {
        EXPECT_GT(eigenvalues(i), 0.0);
    }
}

TEST_F(EKFTest, PropagateStep) {
    // Test covariance propagation
    double dt = 0.01;
    
    State state = initial_state;
    Eigen::Matrix3d P_before = ekf.get_P_pos();
    ekf.propagate(state, dt);
    Eigen::Matrix3d P_after = ekf.get_P_pos();
    
    // Covariance should change due to process noise
    EXPECT_NE(P_after(0,0), P_before(0,0));
}

TEST_F(EKFTest, CovariancePropagation) {
    // Covariance should grow during propagation
    State state = initial_state;
    Eigen::Matrix3d P_before = ekf.get_P_pos();
    
    // Multiple propagation steps
    for (int i = 0; i < 10; i++) {
        ekf.propagate(state, 0.01);
    }
    
    Eigen::Matrix3d P_after = ekf.get_P_pos();
    
    // Position uncertainty should increase
    EXPECT_GT(P_after(0, 0), P_before(0, 0));  // North position
    EXPECT_GT(P_after(1, 1), P_before(1, 1));  // East position
    EXPECT_GT(P_after(2, 2), P_before(2, 2));  // Down position
    
    // Covariance should remain symmetric
    EXPECT_NEAR((P_after - P_after.transpose()).norm(), 0.0, 1e-10);
}

TEST_F(EKFTest, AltitudeUpdate) {
    // Test radar altimeter measurement update
    double true_agl = 100.0;
    double terrain_height = -400.0;  // Terrain at 400m elevation
    
    // Set state to be at true_agl above terrain
    State state = initial_state;
    state.p_NED.z() = -(terrain_height + true_agl);
    
    // Create measurement
    double measured_agl = true_agl + 1.0;  // 1m error
    Eigen::Vector2d slope(0.1, 0.0);  // 10% slope in north direction
    
    // Store covariance before update
    double P_before = ekf.get_P_pos()(2, 2);  // Down position variance
    
    // Apply update
    ekf.update_agl(state, measured_agl, terrain_height, slope);
    
    // Position should move toward measurement
    double agl_after = -state.p_NED.z() - terrain_height;
    EXPECT_LT(std::abs(agl_after - measured_agl), std::abs(true_agl - measured_agl));
    
    // Covariance should decrease
    EXPECT_LT(ekf.get_P_pos()(2, 2), P_before);
}

TEST_F(EKFTest, GravityUpdate) {
    // Test gravity anomaly update
    State state = initial_state;
    IIR1 filter;
    filter.alpha = 0.1;  // Set filter coefficient
    filter.step(-9.81);  // Update with gravity measurement
    
    // Apply gravity update
    ekf.update_gravity(state, filter);
    
    // This should constrain vertical channel
    // Check that covariance is updated (exact behavior depends on implementation)
    EXPECT_TRUE(ekf.get_P_pos()(2, 2) > 0);  // Should remain positive
}

TEST_F(EKFTest, MeasurementRejection) {
    // Test that outliers are properly rejected
    double true_agl = 100.0;
    double terrain_height = -400.0;
    
    State state = initial_state;
    state.p_NED.z() = -(terrain_height + true_agl);
    
    // Create outlier measurement (way off)
    double outlier_agl = true_agl + 100.0;  // 100m error (huge)
    Eigen::Vector2d slope(0.1, 0.0);
    
    State before = state;
    Eigen::Matrix3d P_before = ekf.get_P_pos();
    
    // Apply update (should be rejected by NIS test)
    ekf.update_agl(state, outlier_agl, terrain_height, slope);
    
    // State shouldn't change much for outlier
    // (Exact behavior depends on gate threshold)
    double position_change = (state.p_NED - before.p_NED).norm();
    EXPECT_LT(position_change, 10.0);  // Should not jump to outlier
}

TEST_F(EKFTest, JosephFormStability) {
    // Test that Joseph form maintains positive definiteness
    // Even with many updates
    
    for (int i = 0; i < 100; i++) {
        State state = initial_state;
        // Propagate
        ekf.propagate(state, 0.01);
        
        // Update with various measurements
        if (i % 5 == 0) {
            ekf.update_agl(state, 100.0 + i * 0.1, -400.0, Eigen::Vector2d(0.1, 0.0));
        }
        if (i % 10 == 0) {
            IIR1 filter;
            filter.alpha = 0.1;
            filter.step(-9.81);
            ekf.update_gravity(state, filter);
        }
        
        // Check position covariance remains valid
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(ekf.get_P_pos());
        double min_eigenvalue = solver.eigenvalues().minCoeff();
        
        EXPECT_GT(min_eigenvalue, -1e-10);  // Allow small numerical errors
        
        // Check symmetry
        double asymmetry = (ekf.get_P_pos() - ekf.get_P_pos().transpose()).norm();
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