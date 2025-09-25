/**
 * Unit Tests for Mathematical Utilities
 * Focus on numerical stability and correctness
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "utils/math_utils.h"

using namespace NavMath;
using namespace Eigen;

class MathUtilsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up any test fixtures
    }
};

// Test skew-symmetric matrix creation
TEST_F(MathUtilsTest, SkewSymmetricMatrix) {
    Vector3d v(1, 2, 3);
    Matrix3d S = SkewSymmetric::skew(v);

    // Check skew-symmetric property: S^T = -S
    EXPECT_LT((S.transpose() + S).norm(), 1e-10)
        << "Matrix not skew-symmetric";

    // Check specific values
    EXPECT_EQ(S(0, 0), 0);
    EXPECT_EQ(S(1, 1), 0);
    EXPECT_EQ(S(2, 2), 0);
    EXPECT_EQ(S(0, 1), -v.z());
    EXPECT_EQ(S(0, 2), v.y());
    EXPECT_EQ(S(1, 0), v.z());

    // Test vex operation (inverse of skew)
    Vector3d v_recovered = SkewSymmetric::vex(S);
    EXPECT_LT((v_recovered - v).norm(), 1e-10)
        << "Vex operation didn't recover original vector";
}

// Test quaternion exponential map
TEST_F(MathUtilsTest, QuaternionExpMap) {
    // Test zero rotation
    Vector3d phi_zero = Vector3d::Zero();
    Quaterniond q_zero = SO3::expMap(phi_zero);
    EXPECT_NEAR(q_zero.w(), 1.0, 1e-10)
        << "Zero rotation doesn't give identity";
    EXPECT_LT(q_zero.vec().norm(), 1e-10)
        << "Zero rotation has non-zero vector part";

    // Test 90-degree rotation about Z
    Vector3d phi_90z(0, 0, M_PI/2);
    Quaterniond q_90z = SO3::expMap(phi_90z);

    // Expected: cos(45°), 0, 0, sin(45°)
    EXPECT_NEAR(q_90z.w(), cos(M_PI/4), 1e-10);
    EXPECT_NEAR(q_90z.z(), sin(M_PI/4), 1e-10);
}

// Test quaternion logarithmic map
TEST_F(MathUtilsTest, QuaternionLogMap) {
    // Test identity quaternion
    Quaterniond q_identity = Quaterniond::Identity();
    Vector3d phi_identity = SO3::logMap(q_identity);
    EXPECT_LT(phi_identity.norm(), 1e-10)
        << "Identity quaternion doesn't give zero rotation";

    // Test round-trip: exp(log(q)) = q
    Quaterniond q(0.9239, 0.3827, 0, 0);  // 45 deg yaw
    Vector3d phi = SO3::logMap(q);
    Quaterniond q_recovered = SO3::expMap(phi);

    double angle_diff = q.angularDistance(q_recovered);
    EXPECT_LT(angle_diff, 1e-10)
        << "Round-trip exp(log(q)) failed";
}

// Test quaternion normalization
TEST_F(MathUtilsTest, QuaternionNormalization) {
    // Create unnormalized quaternion
    Quaterniond q(2, 3, 4, 5);

    Quaterniond q_norm = SO3::normalizeQuaternion(q);

    EXPECT_NEAR(q_norm.norm(), 1.0, 1e-10)
        << "Quaternion not properly normalized";

    // Check direction preserved
    double scale = q.norm();
    Quaterniond expected = q;
    expected.coeffs() /= scale;

    EXPECT_LT((q_norm.coeffs() - expected.coeffs()).norm(), 1e-10)
        << "Quaternion direction not preserved";
}

// Test quaternion boxplus operation
TEST_F(MathUtilsTest, QuaternionBoxplus) {
    Quaterniond q = Quaterniond::Identity();
    Vector3d delta(0.1, 0, 0);  // Small rotation about X

    Quaterniond q_new = SO3::boxplus(q, delta);

    // Should have rotated slightly
    EXPECT_GT(q.angularDistance(q_new), 0.09)
        << "Boxplus didn't rotate enough";
    EXPECT_LT(q.angularDistance(q_new), 0.11)
        << "Boxplus rotated too much";
}

// Test quaternion integration
TEST_F(MathUtilsTest, QuaternionIntegration) {
    Quaterniond q = Quaterniond::Identity();
    Vector3d omega(0, 0, 1.0);  // 1 rad/s about Z
    double dt = 0.1;

    Quaterniond q_new = SO3::integrateQuaternion(q, omega, dt);

    // Should have rotated by 0.1 radians
    auto euler = q_new.toRotationMatrix().eulerAngles(0, 1, 2);
    EXPECT_NEAR(std::abs(euler(2)), 0.1, 0.001)
        << "Quaternion integration incorrect";
}

// Test gravity model
TEST_F(MathUtilsTest, GravityModel) {
    // Test at equator, sea level
    Vector3d g_eq = EarthModel::gravityNED(0, 0);
    EXPECT_NEAR(g_eq.z(), 9.78, 0.1)  // Equatorial gravity
        << "Gravity at equator incorrect";
    EXPECT_NEAR(g_eq.x(), 0, 1e-6);
    EXPECT_NEAR(g_eq.y(), 0, 1e-6);

    // Test at poles
    Vector3d g_pole = EarthModel::gravityNED(M_PI/2, 0);
    EXPECT_NEAR(g_pole.z(), 9.83, 0.1)  // Polar gravity
        << "Gravity at pole incorrect";

    // Test altitude variation
    Vector3d g_high = EarthModel::gravityNED(0, 10000);
    EXPECT_LT(g_high.z(), g_eq.z())
        << "Gravity should decrease with altitude";
}

// Test Earth rotation rate
TEST_F(MathUtilsTest, EarthRate) {
    // At equator
    Vector3d omega_eq = EarthModel::earthRate(0);
    EXPECT_NEAR(omega_eq.x(), EARTH_RATE, 1e-10);
    EXPECT_NEAR(omega_eq.y(), 0, 1e-10);
    EXPECT_NEAR(omega_eq.z(), 0, 1e-10);

    // At 45 degrees latitude
    Vector3d omega_45 = EarthModel::earthRate(M_PI/4);
    EXPECT_NEAR(omega_45.x(), EARTH_RATE * cos(M_PI/4), 1e-10);
    EXPECT_NEAR(omega_45.z(), -EARTH_RATE * sin(M_PI/4), 1e-10);
}

// Test ECEF to geodetic conversion
TEST_F(MathUtilsTest, ECEFToGeodetic) {
    // Point on equator at prime meridian
    Vector3d ecef(WGS84_A, 0, 0);
    Vector3d lla = EarthModel::ecefToGeodetic(ecef);

    EXPECT_NEAR(lla(0), 0, 1e-9)  // Latitude
        << "Latitude conversion incorrect";
    EXPECT_NEAR(lla(1), 0, 1e-9)  // Longitude
        << "Longitude conversion incorrect";
    EXPECT_NEAR(lla(2), 0, 1.0)   // Height (within 1m)
        << "Height conversion incorrect";
}

// Test matrix utilities
TEST_F(MathUtilsTest, MatrixUtilities) {
    // Test positive definiteness check
    Matrix3d P_good = Matrix3d::Identity();
    // Test makePositiveDefinite instead
    Matrix3d P_bad;
    P_bad << 1, 2, 0,
             2, 1, 0,
             0, 0, 1;  // Not positive definite
    MatrixXd P_fixed = MatrixUtils::makePositiveDefinite(P_bad);

    // Check that eigenvalues are positive
    Eigen::SelfAdjointEigenSolver<MatrixXd> es(P_fixed);
    EXPECT_GT(es.eigenvalues().minCoeff(), 0)
        << "Matrix not made positive definite";

    // Test matrix square root
    Matrix3d P;
    P << 4, 2, 1,
         2, 5, 2,
         1, 2, 6;

    MatrixXd S = MatrixUtils::matrixSquareRoot(P);
    MatrixXd P_recovered = S * S.transpose();

    EXPECT_LT((P_recovered - P).norm(), 1e-10)
        << "Matrix square root incorrect";
}

// Test Cholesky operations
TEST_F(MathUtilsTest, CholeskyOperations) {
    // Create a simple positive definite matrix
    MatrixXd L = MatrixXd::Identity(3, 3);
    L(1, 0) = 0.5;
    L(2, 0) = 0.3;
    L(2, 1) = 0.4;

    VectorXd v(3);
    v << 1, 0, 0;

    // Test Cholesky update
    double sigma = 1.0;
    MatrixXd L_updated = MatrixUtils::choleskyUpdate(L, v, sigma);

    // Updated matrix should still be triangular
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            EXPECT_NEAR(L_updated(i, j), 0.0, 1e-10)
                << "Updated matrix not triangular";
        }
    }
}

// Test statistical functions
TEST_F(MathUtilsTest, StatisticalFunctions) {
    // Test NIS computation
    VectorXd innovation(2);
    innovation << 1, 2;

    MatrixXd S(2, 2);
    S << 2, 0,
         0, 2;

    double nis = Statistics::computeNIS(innovation, S);
    EXPECT_NEAR(nis, 2.5, 1e-10)  // (1^2 + 2^2) / 2 = 2.5
        << "NIS calculation incorrect";

    // Test Mahalanobis distance
    VectorXd x(2);
    x << 3, 4;
    VectorXd mean(2);
    mean << 1, 2;
    MatrixXd cov(2, 2);
    cov << 1, 0,
           0, 1;

    double dist = Statistics::mahalanobisDistance(x, mean, cov);
    EXPECT_NEAR(dist, sqrt(8.0), 1e-10)  // sqrt((2^2 + 2^2))
        << "Mahalanobis distance incorrect";
}

// Test gravity tensor operations
TEST_F(MathUtilsTest, GravityTensorOperations) {
    // Create a symmetric trace-free tensor
    Matrix3d T;
    T << 1, 0, 0,
         0, -0.5, 0,
         0, 0, -0.5;

    // Test conversion to STF
    Eigen::Matrix<double, 5, 1> stf = GravityTensor::tensorToSTF(T);
    EXPECT_EQ(stf.size(), 5)
        << "STF vector wrong size";

    // Test round-trip conversion
    Matrix3d T_recovered = GravityTensor::stfToTensor(stf);
    EXPECT_LT((T_recovered - T).norm(), 1e-10)
        << "Tensor round-trip conversion failed";

    // Test ensuring STF property
    Matrix3d T_not_stf = Matrix3d::Random();
    Matrix3d T_stf = GravityTensor::makeSTF(T_not_stf);

    // Check trace-free
    EXPECT_NEAR(T_stf.trace(), 0.0, 1e-10)
        << "Tensor not trace-free";

    // Check symmetric
    EXPECT_LT((T_stf - T_stf.transpose()).norm(), 1e-10)
        << "Tensor not symmetric";
}

// Test covariance operations
TEST_F(MathUtilsTest, CovarianceOperations) {
    // Test enforcing symmetry
    Matrix3d P;
    P << 1, 0.5, 0.2,
         0.49, 2, 0.3,  // Slightly asymmetric
         0.21, 0.3, 1.5;

    MatrixXd P_sym = MatrixUtils::makeSymmetric(P);

    EXPECT_LT((P_sym - P_sym.transpose()).norm(), 1e-10)
        << "Matrix not symmetric after enforcement";

    // Test enforcing positive definiteness
    Matrix3d P_bad = Matrix3d::Identity();
    P_bad(1, 1) = -1;  // Make it non-PD

    MatrixXd P_fixed = MatrixUtils::makePositiveDefinite(P_bad, 1e-6);

    // Check eigenvalues are positive
    Eigen::SelfAdjointEigenSolver<MatrixXd> es(P_fixed);
    EXPECT_GT(es.eigenvalues().minCoeff(), 0)
        << "Matrix not positive definite after enforcement";
}

// Test transport rate calculation
TEST_F(MathUtilsTest, TransportRate) {
    // Stationary at equator
    Vector3d vel_zero = Vector3d::Zero();
    Vector3d transport = EarthModel::transportRate(vel_zero, 0, 0);

    EXPECT_LT(transport.norm(), 1e-10)
        << "Non-zero transport rate for stationary object";

    // Moving north at 100 m/s
    Vector3d vel_north(100, 0, 0);
    transport = EarthModel::transportRate(vel_north, 0, 0);

    EXPECT_GT(transport.y(), 0)  // Should have eastward component
        << "Transport rate incorrect for northward motion";
}

// Test extreme value handling
TEST_F(MathUtilsTest, ExtremeValues) {
    // Test quaternion normalization with tiny values
    Quaterniond q_tiny(1e-100, 1e-100, 1e-100, 1e-100);
    Quaterniond q_norm = SO3::normalizeQuaternion(q_tiny);

    EXPECT_TRUE(std::isfinite(q_norm.w()))
        << "Quaternion normalization failed with tiny values";
    EXPECT_NEAR(q_norm.norm(), 1.0, 1e-10)
        << "Quaternion not normalized with tiny values";

    // Test with large values
    Quaterniond q_large(1e100, 0, 0, 0);
    q_norm = SO3::normalizeQuaternion(q_large);

    EXPECT_NEAR(q_norm.norm(), 1.0, 1e-10)
        << "Quaternion not normalized with large values";
}