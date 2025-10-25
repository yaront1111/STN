// Property tests for mathematical invariants
// Verifies covariance PSD, SO(3) properties, coordinate consistency

#include <gtest/gtest.h>
#include "core/ukf.h"
#include "core/watchdogs.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>

using namespace chimera::core;

// =================== Covariance PSD Property Tests ===================
class CovariancePSDTest : public ::testing::Test {
 protected:
  static bool isPSD(const Eigen::MatrixXd& M) {
    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    return ldlt.isPositive();
  }

  static bool isSymmetric(const Eigen::MatrixXd& M, double tol = 1e-9) {
    return M.isApprox(M.transpose(), tol);
  }
};

TEST_F(CovariancePSDTest, IdentityIsPSD) {
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6);
  EXPECT_TRUE(isSymmetric(cov));
  EXPECT_TRUE(isPSD(cov));
}

TEST_F(CovariancePSDTest, DiagonalPositiveIsPSD) {
  Eigen::VectorXd diag(6);
  diag << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  Eigen::MatrixXd cov = diag.asDiagonal();

  EXPECT_TRUE(isSymmetric(cov));
  EXPECT_TRUE(isPSD(cov));
}

TEST_F(CovariancePSDTest, CorrelatedIsPSD) {
  // Create valid covariance with correlation
  Eigen::Matrix3d L;
  L << 1.0, 0.0, 0.0,
       0.5, 0.8, 0.0,
       0.2, 0.3, 0.9;
  Eigen::Matrix3d cov = L * L.transpose();  // Guaranteed PSD

  EXPECT_TRUE(isSymmetric(cov));
  EXPECT_TRUE(isPSD(cov));
}

TEST_F(CovariancePSDTest, NegativeEigenvalueNotPSD) {
  Eigen::Matrix3d cov;
  cov << 1.0, 0.0, 0.0,
         0.0, -0.5, 0.0,  // Negative eigenvalue
         0.0, 0.0, 1.0;

  EXPECT_TRUE(isSymmetric(cov));
  EXPECT_FALSE(isPSD(cov));
}

TEST_F(CovariancePSDTest, UKFCovarianceInvariant) {
  // Property: UKF covariance must always be PSD
  UKF::NoiseParams noise;
  noise.accel_noise_density = 0.01;
  noise.gyro_noise_density = 0.001;
  noise.accel_random_walk = 0.0001;
  noise.gyro_random_walk = 0.00001;

  UKF ukf(noise);

  // Initialize with valid PSD covariance
  gtsam::Pose3 pose0(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
  Eigen::Vector3d vel0 = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity();
  ukf.initialize(pose0, vel0, P0);

  // Property: Initial covariance must be PSD
  auto cov0 = ukf.getCovariance();
  EXPECT_TRUE(isSymmetric(cov0));
  EXPECT_TRUE(isPSD(cov0));

  // Propagate with IMU
  UKF::ImuMeasurement imu;
  imu.timestamp = 0.01;
  imu.accel = Eigen::Vector3d(0, 0, -9.81);  // Hovering
  imu.gyro = Eigen::Vector3d::Zero();
  imu.temperature = 25.0;

  ukf.propagate(imu);

  // Property: Covariance after propagation must remain PSD
  auto cov1 = ukf.getCovariance();
  EXPECT_TRUE(isSymmetric(cov1));
  EXPECT_TRUE(isPSD(cov1));
}

// =================== SO(3) Property Tests ===================
class RotationMatrixTest : public ::testing::Test {
 protected:
  static bool isOrthogonal(const Eigen::Matrix3d& R, double tol = 1e-6) {
    Eigen::Matrix3d I = R * R.transpose();
    return I.isApprox(Eigen::Matrix3d::Identity(), tol);
  }

  static bool hasUnitDeterminant(const Eigen::Matrix3d& R, double tol = 1e-6) {
    return std::abs(R.determinant() - 1.0) < tol;
  }

  static bool isSO3(const Eigen::Matrix3d& R, double tol = 1e-6) {
    return isOrthogonal(R, tol) && hasUnitDeterminant(R, tol);
  }
};

TEST_F(RotationMatrixTest, IdentityIsSO3) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  EXPECT_TRUE(isSO3(R));
}

TEST_F(RotationMatrixTest, RotationAboutZIsSO3) {
  double theta = M_PI / 4;  // 45 degrees
  Eigen::Matrix3d R;
  R << cos(theta), -sin(theta), 0,
       sin(theta),  cos(theta), 0,
       0,           0,          1;

  EXPECT_TRUE(isSO3(R));
}

TEST_F(RotationMatrixTest, GTSAMRot3IsSO3) {
  // Arbitrary rotation
  gtsam::Rot3 rot = gtsam::Rot3::Ypr(0.5, 0.3, 0.1);
  Eigen::Matrix3d R = rot.matrix();

  EXPECT_TRUE(isSO3(R));
}

TEST_F(RotationMatrixTest, CompositionPreservesSO3) {
  // Property: R1, R2 ∈ SO(3) => R1 * R2 ∈ SO(3)
  gtsam::Rot3 r1 = gtsam::Rot3::Ypr(0.5, 0.2, 0.1);
  gtsam::Rot3 r2 = gtsam::Rot3::Ypr(0.3, 0.4, 0.2);
  gtsam::Rot3 r3 = r1.compose(r2);

  EXPECT_TRUE(isSO3(r1.matrix()));
  EXPECT_TRUE(isSO3(r2.matrix()));
  EXPECT_TRUE(isSO3(r3.matrix()));
}

TEST_F(RotationMatrixTest, InversePreservesSO3) {
  // Property: R ∈ SO(3) => R^{-1} ∈ SO(3)
  gtsam::Rot3 r = gtsam::Rot3::Ypr(0.5, 0.3, 0.1);
  gtsam::Rot3 r_inv = r.inverse();

  EXPECT_TRUE(isSO3(r.matrix()));
  EXPECT_TRUE(isSO3(r_inv.matrix()));

  // Property: R * R^{-1} = I
  Eigen::Matrix3d I = r.matrix() * r_inv.matrix();
  EXPECT_TRUE(I.isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST_F(RotationMatrixTest, ScaledMatrixNotSO3) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 2.0;  // Scaled
  EXPECT_FALSE(isSO3(R));
}

// =================== Coordinate Consistency Tests ===================
class CoordinateConsistencyTest : public ::testing::Test {
 protected:
  static constexpr double g = 9.81;
  static constexpr double tol = 1e-6;
};

TEST_F(CoordinateConsistencyTest, NEDGravityDown) {
  // In NED frame, gravity points down (+Z)
  Eigen::Vector3d g_NED(0, 0, g);

  EXPECT_NEAR(g_NED.x(), 0.0, tol);
  EXPECT_NEAR(g_NED.y(), 0.0, tol);
  EXPECT_GT(g_NED.z(), 0.0);  // Down is positive
}

TEST_F(CoordinateConsistencyTest, HoveringSpecificForce) {
  // Hovering vehicle in NED: accel = -g_NED (upward reaction force)
  Eigen::Vector3d a_hover(0, 0, -g);
  Eigen::Vector3d g_NED(0, 0, g);

  // Net acceleration should be zero
  Eigen::Vector3d a_net = a_hover + g_NED;
  EXPECT_NEAR(a_net.norm(), 0.0, tol);
}

TEST_F(CoordinateConsistencyTest, AltitudeNEDConvention) {
  // In NED: altitude above ground = -Z
  // If drone is 10m above ground, Z = -10m
  double altitude_agl = 10.0;  // meters above ground
  double z_ned = -altitude_agl;

  EXPECT_LT(z_ned, 0.0);  // Above ground => negative Z
  EXPECT_NEAR(z_ned, -10.0, tol);
}

TEST_F(CoordinateConsistencyTest, RotationPreservesGravityAlignment) {
  // Property: For hovering, R * a_body + g_NED ≈ 0
  // where a_body is measured IMU acceleration

  // Body frame acceleration (hovering, IMU measures upward)
  Eigen::Vector3d a_body(0, 0, -g);  // FRD: upward is -Z

  // Rotation from body to NED (identity for level flight)
  gtsam::Rot3 R_NED_body = gtsam::Rot3();

  // Transform to NED
  Eigen::Vector3d a_NED = R_NED_body.matrix() * a_body;
  Eigen::Vector3d g_NED(0, 0, g);

  // Net acceleration should be zero (hovering)
  Eigen::Vector3d residual = a_NED + g_NED;
  EXPECT_NEAR(residual.norm(), 0.0, tol);
}

TEST_F(CoordinateConsistencyTest, VelocityIntegrationConsistency) {
  // Property: velocity integration must respect coordinate frame
  double dt = 0.01;  // 10ms
  Eigen::Vector3d vel0(1.0, 0.0, 0.0);  // 1 m/s north
  Eigen::Vector3d accel(0.0, 0.0, g);   // gravity down

  // Simple Euler integration
  Eigen::Vector3d vel1 = vel0 + accel * dt;

  // North velocity unchanged
  EXPECT_NEAR(vel1.x(), 1.0, tol);
  EXPECT_NEAR(vel1.y(), 0.0, tol);

  // Downward velocity increases (positive Z)
  EXPECT_GT(vel1.z(), 0.0);
  EXPECT_NEAR(vel1.z(), g * dt, tol);
}

// =================== Watchdog Property Tests ===================
class WatchdogInvariantTest : public ::testing::Test {};

TEST_F(WatchdogInvariantTest, PSDCovarianceNeverRejected) {
  // Property: Valid PSD covariance should always pass health check
  CovarianceHealthMonitor monitor;

  // Create guaranteed PSD matrix (Cholesky decomposition)
  Eigen::Matrix3d L;
  L << 1.0, 0.0, 0.0,
       0.5, 0.8, 0.0,
       0.2, 0.3, 0.9;
  Eigen::Matrix3d cov = L * L.transpose();

  auto health = monitor.evaluate(cov);

  EXPECT_TRUE(health.is_healthy);
  EXPECT_TRUE(health.is_psd);
  EXPECT_TRUE(health.is_well_conditioned);
  EXPECT_FALSE(health.is_diverging);
}

TEST_F(WatchdogInvariantTest, FiniteValuesNeverTriggerNaN) {
  // Property: All finite values should pass NaN guard
  std::vector<double> test_values = {
    0.0, 1.0, -1.0, 1e-10, 1e10, -1e10
  };

  for (double val : test_values) {
    auto result = NaNGuard::check(val, "test");
    EXPECT_TRUE(result.is_valid) << "Failed for value: " << val;
  }
}

TEST_F(WatchdogInvariantTest, MonotonicTimeNeverGoesBackward) {
  // Property: Monotonic clock should reject any backwards time jump
  MonotonicClock clock;

  clock.check(1.0, "sensor");
  clock.check(2.0, "sensor");

  auto result = clock.check(1.5, "sensor");  // Backwards
  EXPECT_FALSE(result.is_valid);
  EXPECT_LT(result.time_jump, 0.0);
}
