/**
 * Unit tests for quaternion operations
 * Critical for INS attitude propagation
 */
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include "types.h"

class QuaternionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Identity quaternion
        q_identity = Eigen::Quaterniond::Identity();
        
        // 90 degree rotations
        q_90z = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
        q_90y = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
        q_90x = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    }
    
    Eigen::Quaterniond q_identity;
    Eigen::Quaterniond q_90z, q_90y, q_90x;
    
    // Helper: Check if quaternions are approximately equal
    bool quaternionsEqual(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double tol = 1e-10) {
        // Handle quaternion double cover (q and -q represent same rotation)
        return (q1.coeffs() - q2.coeffs()).norm() < tol || 
               (q1.coeffs() + q2.coeffs()).norm() < tol;
    }
};

TEST_F(QuaternionTest, Identity) {
    // Identity quaternion should not change vectors
    Eigen::Vector3d v(1, 2, 3);
    Eigen::Vector3d v_rot = q_identity * v;
    
    EXPECT_NEAR((v - v_rot).norm(), 0.0, 1e-10);
}

TEST_F(QuaternionTest, Normalization) {
    // Quaternions must stay normalized
    Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);  // Not normalized
    q.normalize();
    
    EXPECT_NEAR(q.norm(), 1.0, 1e-10);
}

TEST_F(QuaternionTest, Rotation90Degrees) {
    // Test 90 degree rotation about Z axis
    Eigen::Vector3d v_x(1, 0, 0);
    Eigen::Vector3d v_rotated = q_90z * v_x;
    Eigen::Vector3d v_expected(0, 1, 0);
    
    EXPECT_NEAR((v_rotated - v_expected).norm(), 0.0, 1e-10);
}

TEST_F(QuaternionTest, CompositionOrder) {
    // Test that quaternion multiplication order matters
    Eigen::Quaterniond q_yz = q_90y * q_90z;
    Eigen::Quaterniond q_zy = q_90z * q_90y;
    
    EXPECT_FALSE(quaternionsEqual(q_yz, q_zy));
    
    // Apply to test vector
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_yz = q_yz * v;
    Eigen::Vector3d v_zy = q_zy * v;
    
    EXPECT_GT((v_yz - v_zy).norm(), 0.1);  // Results should be different
}

TEST_F(QuaternionTest, Inverse) {
    // Test quaternion inverse
    Eigen::Quaterniond q_inv = q_90z.inverse();
    Eigen::Quaterniond q_identity_test = q_90z * q_inv;
    
    EXPECT_TRUE(quaternionsEqual(q_identity_test, q_identity));
}

TEST_F(QuaternionTest, EulerConversion) {
    // Test conversion to/from Euler angles
    double roll = 0.1;
    double pitch = 0.2;
    double yaw = 0.3;
    
    // Create quaternion from Euler angles (ZYX convention - yaw, pitch, roll)
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    // Convert back to Euler
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order
    
    EXPECT_NEAR(euler[0], yaw, 1e-10);
    EXPECT_NEAR(euler[1], pitch, 1e-10);
    EXPECT_NEAR(euler[2], roll, 1e-10);
}

TEST_F(QuaternionTest, SmallAngleApproximation) {
    // For small angles, quaternion should be approximately [1, theta/2]
    double small_angle = 0.001;  // radians
    Eigen::Quaterniond q_small(Eigen::AngleAxisd(small_angle, Eigen::Vector3d::UnitZ()));
    
    EXPECT_NEAR(q_small.w(), 1.0, 1e-6);
    EXPECT_NEAR(q_small.z(), small_angle/2, 1e-9);
}

TEST_F(QuaternionTest, IntegrationStability) {
    // Test that repeated small rotations stay normalized
    Eigen::Quaterniond q = q_identity;
    Eigen::Vector3d omega(0.1, 0.05, 0.02);  // rad/s
    double dt = 0.01;
    
    for (int i = 0; i < 1000; i++) {
        // Simple quaternion integration (first-order)
        Eigen::Quaterniond q_dot;
        q_dot.w() = 0.5 * (-omega.x()*q.x() - omega.y()*q.y() - omega.z()*q.z());
        q_dot.x() = 0.5 * ( omega.x()*q.w() + omega.z()*q.y() - omega.y()*q.z());
        q_dot.y() = 0.5 * ( omega.y()*q.w() - omega.z()*q.x() + omega.x()*q.z());
        q_dot.z() = 0.5 * ( omega.z()*q.w() + omega.y()*q.x() - omega.x()*q.y());
        
        q.w() += q_dot.w() * dt;
        q.x() += q_dot.x() * dt;
        q.y() += q_dot.y() * dt;
        q.z() += q_dot.z() * dt;
        
        q.normalize();  // Critical for stability
    }
    
    EXPECT_NEAR(q.norm(), 1.0, 1e-10);
}