/**
 * Unit tests for utility functions from chimera_node_multi.cpp
 * Tests helper functions: median, medianAccel, effectiveDepthMeters_AGL, etc.
 */

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace gtsam;

// ===== Replicate utility functions from chimera_node_multi.cpp =====

static inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    return v[v.size()/2];
}

struct ImuSample {
    double t;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    Eigen::Vector4d quat;
};

static Eigen::Vector3d medianAccel(const std::vector<ImuSample>& imu, double t0, double horizon=0.5) {
    std::vector<Eigen::Vector3d> w;
    for (const auto& s : imu) {
        if (s.t - t0 > horizon) break;
        w.push_back(s.accel);
    }
    if (w.empty()) return Eigen::Vector3d(0,0,-9.81);
    // component-wise median
    auto med = [&](int k){
        std::vector<double> v;
        v.reserve(w.size());
        for (auto& a : w) v.push_back(a(k));
        std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
        return v[v.size()/2];
    };
    return Eigen::Vector3d(med(0), med(1), med(2));
}

static double effectiveDepthMeters_AGL_helper(const Pose3& x_wb, const Matrix3& R_bc, double agl_m) {
    // R_wb = body in world; R_wc = camera in world
    const Matrix3 R_wb = x_wb.rotation().matrix();
    const Matrix3 R_wc = R_wb * R_bc;
    // Camera optical axis in world (camera Z-forward)
    const Eigen::Vector3d ez_cam_world = R_wc.col(2);
    // Ground normal (world up)
    const Eigen::Vector3d n_world(0, 0, 1);
    const double cos_incidence = std::abs(n_world.dot(ez_cam_world)); // |cos(theta)|
    const double clamp_val = std::max(0.15, std::min(1.0, cos_incidence)); // avoid crazy tilt
    return agl_m / clamp_val; // Z_eff = h / cos(theta)
}

static inline double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    return angle - M_PI;
}

// ===== Test Suite: Median Function =====

class MedianTest : public ::testing::Test {};

TEST_F(MedianTest, OddCount) {
    std::vector<double> v = {5.0, 1.0, 3.0, 9.0, 7.0};
    EXPECT_DOUBLE_EQ(median(v), 5.0);
}

TEST_F(MedianTest, EvenCount) {
    // nth_element for even count returns one of the two middle elements
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0};
    double result = median(v);
    // Result should be either 2.0 or 3.0 (implementation dependent)
    EXPECT_TRUE(result == 2.0 || result == 3.0);
}

TEST_F(MedianTest, SingleElement) {
    std::vector<double> v = {42.0};
    EXPECT_DOUBLE_EQ(median(v), 42.0);
}

TEST_F(MedianTest, EmptyVector) {
    std::vector<double> v;
    EXPECT_DOUBLE_EQ(median(v), 0.0);
}

TEST_F(MedianTest, AllSame) {
    std::vector<double> v = {7.5, 7.5, 7.5, 7.5};
    EXPECT_DOUBLE_EQ(median(v), 7.5);
}

TEST_F(MedianTest, WithNegatives) {
    std::vector<double> v = {-10.0, -5.0, 0.0, 5.0, 10.0};
    EXPECT_DOUBLE_EQ(median(v), 0.0);
}

// ===== Test Suite: MedianAccel Function =====

class MedianAccelTest : public ::testing::Test {
protected:
    std::vector<ImuSample> imu_data;

    void SetUp() override {
        // Create 10 IMU samples with varying accelerations
        for (int i = 0; i < 10; ++i) {
            ImuSample sample;
            sample.t = i * 0.1;  // 0.0, 0.1, 0.2, ..., 0.9
            // Add some noise around gravity
            sample.accel = Eigen::Vector3d(
                (i - 5) * 0.1,      // X: -0.5 to 0.4
                (i - 5) * 0.05,     // Y: -0.25 to 0.2
                -9.81 + (i % 3) * 0.02  // Z: gravity with small noise
            );
            sample.gyro = Eigen::Vector3d::Zero();
            sample.quat = Eigen::Vector4d(0, 0, 0, 1);
            imu_data.push_back(sample);
        }
    }
};

TEST_F(MedianAccelTest, NormalWindow) {
    // Median over first 5 samples (t=0.0 to t=0.4)
    Eigen::Vector3d result = medianAccel(imu_data, 0.0, 0.5);

    // X component median of [-0.5, -0.4, -0.3, -0.2, -0.1]
    // nth_element for even count may return either of the two middle elements
    EXPECT_GT(result(0), -0.4);  // Should be > -0.4
    EXPECT_LT(result(0), -0.1);  // Should be < -0.1

    // Z should be close to -9.81
    EXPECT_NEAR(result(2), -9.81, 0.05);
}

TEST_F(MedianAccelTest, EmptyWindow) {
    // Query before any data
    Eigen::Vector3d result = medianAccel(imu_data, -10.0, 0.1);

    // Should return default gravity vector
    EXPECT_DOUBLE_EQ(result(0), 0.0);
    EXPECT_DOUBLE_EQ(result(1), 0.0);
    EXPECT_DOUBLE_EQ(result(2), -9.81);
}

TEST_F(MedianAccelTest, SmallWindow) {
    // medianAccel includes all samples from start until s.t - t0 > horizon
    // Query at t=0.0 with tiny horizon to get only first sample
    Eigen::Vector3d result = medianAccel(imu_data, 0.0, 0.05);

    // Should get just i=0: X = (0-5)*0.1 = -0.5
    EXPECT_NEAR(result(0), -0.5, 0.01);
    // Y = (0-5)*0.05 = -0.25
    EXPECT_NEAR(result(1), -0.25, 0.01);
    // Z should be close to -9.81
    EXPECT_NEAR(result(2), -9.81, 0.05);
}

// NOTE: Effective depth tests moved to test_depth_algorithms.cpp to avoid duplication

// ===== Test Suite: Angle Wrapping =====

class AngleWrapTest : public ::testing::Test {};

TEST_F(AngleWrapTest, NoWrap) {
    // Angles already in [-π, π] should remain unchanged
    EXPECT_NEAR(wrapToPi(0.0), 0.0, 1e-9);
    EXPECT_NEAR(wrapToPi(M_PI / 2), M_PI / 2, 1e-9);
    EXPECT_NEAR(wrapToPi(-M_PI / 2), -M_PI / 2, 1e-9);
}

TEST_F(AngleWrapTest, WrapPositive) {
    // 3π wraps to π or -π (both are equivalent on the circle)
    double result = wrapToPi(3.0 * M_PI);
    EXPECT_TRUE(std::abs(result - M_PI) < 1e-6 || std::abs(result + M_PI) < 1e-6);

    // 2π should wrap to 0 (or very close)
    EXPECT_NEAR(wrapToPi(2.0 * M_PI), 0.0, 1e-6);
}

TEST_F(AngleWrapTest, WrapNegative) {
    // -3π should wrap to -π
    EXPECT_NEAR(wrapToPi(-3.0 * M_PI), -M_PI, 1e-6);

    // -2π should wrap to 0
    EXPECT_NEAR(wrapToPi(-2.0 * M_PI), 0.0, 1e-6);
}

TEST_F(AngleWrapTest, BoundaryPi) {
    // π and -π are both valid and equivalent
    double result = wrapToPi(M_PI);
    EXPECT_TRUE(std::abs(result - M_PI) < 1e-6 || std::abs(result + M_PI) < 1e-6);
}

// ===== Test Suite: Rotation Matrix Validation =====

class RotationMatrixTest : public ::testing::Test {};

TEST_F(RotationMatrixTest, Orthogonality) {
    // Test that GTSAM Rot3 matrices are orthogonal (R^T * R = I)
    Rot3 R = Rot3::Ypr(0.5, 0.3, -0.2);  // Random yaw, pitch, roll
    Matrix3 mat = R.matrix();
    Matrix3 should_be_identity = mat.transpose() * mat;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            EXPECT_NEAR(should_be_identity(i, j), expected, 1e-9);
        }
    }
}

TEST_F(RotationMatrixTest, Determinant) {
    // Rotation matrices must have det(R) = 1
    Rot3 R = Rot3::Roll(0.7) * Rot3::Pitch(0.4) * Rot3::Yaw(1.2);
    Matrix3 mat = R.matrix();
    double det = mat.determinant();

    EXPECT_NEAR(det, 1.0, 1e-9);
}

TEST_F(RotationMatrixTest, InverseEqualsTranspose) {
    // For rotation matrices: R^-1 = R^T
    Rot3 R = Rot3::Ypr(1.0, 0.5, -0.3);
    Matrix3 mat = R.matrix();
    Matrix3 inv = R.inverse().matrix();
    Matrix3 transpose = mat.transpose();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(inv(i, j), transpose(i, j), 1e-9);
        }
    }
}

// ===== Test Suite: Quaternion Helpers =====

class QuaternionTest : public ::testing::Test {};

TEST_F(QuaternionTest, IdentityQuaternion) {
    // Identity quaternion (xyzw): [0, 0, 0, 1]
    Rot3 R_identity = Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);  // (w,x,y,z)

    // Should result in identity rotation
    Matrix3 mat = R_identity.matrix();
    EXPECT_TRUE(mat.isIdentity(1e-9));
}

TEST_F(QuaternionTest, NormalizationRequired) {
    // GTSAM should handle normalization automatically
    Rot3 R = Rot3::Quaternion(2.0, 0.0, 0.0, 0.0);  // Unnormalized (w,x,y,z)

    // Should still give identity (after normalization)
    Matrix3 mat = R.matrix();
    EXPECT_TRUE(mat.isIdentity(1e-6));
}

TEST_F(QuaternionTest, QuaternionFromAxisAngle) {
    // 90° rotation around Z-axis
    double angle = M_PI / 2;
    Eigen::Vector3d axis(0, 0, 1);

    Rot3 R = Rot3::AxisAngle(axis, angle);

    // Point along X-axis (1,0,0) should rotate to Y-axis (0,1,0)
    Eigen::Vector3d x_axis(1, 0, 0);
    Eigen::Vector3d rotated = R.matrix() * x_axis;

    EXPECT_NEAR(rotated(0), 0.0, 1e-9);
    EXPECT_NEAR(rotated(1), 1.0, 1e-9);
    EXPECT_NEAR(rotated(2), 0.0, 1e-9);
}

// Note: main() is defined in test_factors.cpp
// All tests are automatically discovered by Google Test
