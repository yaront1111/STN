#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <factors/optical_flow_factor.h>
#include <factors/aero_velocity_factor.h>
#include <factors/aero_velocity_wind_factor.h>
#include <Eigen/Dense>
#include <cmath>

using namespace gtsam;
using namespace chimera::factors;
using symbol_shorthand::X;  // Pose3
using symbol_shorthand::V;  // Velocity
using symbol_shorthand::B;  // Bias

// ===== Helper Functions =====

/**
 * @brief Compute numerical derivative ∂f/∂x using central finite differences
 */
template<typename Func>
Matrix numericalDerivative(Func f, const Vector& x, double h = 1e-5) {
  const int m = f(x).size();  // Output dimension
  const int n = x.size();     // Input dimension
  Matrix J = Matrix::Zero(m, n);

  for (int j = 0; j < n; ++j) {
    Vector x_plus = x;
    Vector x_minus = x;
    x_plus(j) += h;
    x_minus(j) -= h;
    J.col(j) = (f(x_plus) - f(x_minus)) / (2.0 * h);
  }

  return J;
}

/**
 * @brief Create test pose at given position and yaw
 * @param x North position (m)
 * @param y East position (m)
 * @param z Down position (m) - NOTE: In NED, Down is +Z!
 * @param yaw Yaw angle (rad)
 */
Pose3 createTestPose(double x, double y, double z, double yaw = 0.0) {
  Rot3 R = Rot3::Yaw(yaw);
  Point3 t(x, y, z);
  return Pose3(R, t);
}

/**
 * @brief Create test pose at given AGL height (convenience for NED)
 * @param x North position (m)
 * @param y East position (m)
 * @param h_agl Above-ground-level height (m, positive = up)
 * @param yaw Yaw angle (rad)
 * @return Pose3 with correct NED convention (z = -h_agl)
 */
Pose3 createTestPoseAGL(double x, double y, double h_agl, double yaw = 0.0) {
  Rot3 R = Rot3::Yaw(yaw);
  Point3 t(x, y, -h_agl);  // NED: altitude AGL = -Z (Down is +Z)
  return Pose3(R, t);
}

/**
 * @brief Create test bias with given gyro and accel biases
 */
imuBias::ConstantBias createTestBias(const Vector3& gyro_bias, const Vector3& accel_bias) {
  return imuBias::ConstantBias(accel_bias, gyro_bias);
}

/**
 * @brief Nadir mount body→camera rotation (ROS camera frame)
 */
Matrix3 nadirMountRotation() {
  Matrix3 R_bc;
  R_bc <<  0,  1,  0,
          -1,  0,  0,
           0,  0,  1;
  return R_bc;
}


// ===== TEST SUITE 1: Optical Flow Physics =====

/**
 * @brief Test translational flow only at image center
 *
 * Setup:
 * - Velocity: v_body = [1, 0, 0] m/s (forward in body frame)
 * - Angular velocity: ω = [0, 0, 0] rad/s (no rotation)
 * - Altitude: h = 10 m
 * - Camera: nadir mount, focal length f = 600 px
 * - Image location: (x, y) = (0, 0) (image center)
 *
 * Expected flow:
 * - v_cam = R_bc * v_body = [0, -1, 0] m/s (downward in camera frame)
 * - At image center: u̇ = -vx/h, v̇ = -vy/h
 * - u̇ = 0/10 = 0 rad/s → 0 px/s
 * - v̇ = -(-1)/10 = 0.1 rad/s → 60 px/s
 */
TEST(OpticalFlowFactorTest, TranslationOnly_ImageCenter) {
  // Setup
  const double h = 10.0;  // Altitude
  const double focal_length = 600.0;  // pixels
  Matrix3 R_bc = nadirMountRotation();

  // State (zero yaw → body frame = world frame)
  Pose3 pose = createTestPoseAGL(0, 0, h, 0);  // NED: h_agl = 10m
  Vector3 vel_body(1.0, 0.0, 0.0);  // 1 m/s forward in body

  // WORLD-FRAME VELOCITY: Convert from body using R_wb
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());

  // Measurement: raw gyro is zero (no rotation)
  Vector3 gyro_meas = Vector3::Zero();

  // Expected flow in camera frame
  Vector3 v_cam = R_bc * vel_body;  // [0, -1, 0]
  double expected_u_dot = -focal_length * v_cam.x() / h;  // 0
  double expected_v_dot = -focal_length * v_cam.y() / h;  // 60 px/s

  // Create factor with ZERO measured flow (should give error = predicted - measured)
  Vector2 measured_flow(0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Evaluate error with WORLD-FRAME velocity
  Vector error = factor.evaluateError(pose, vel_world, bias);

  // Error should be (predicted - measured) = [expected_u_dot - 0, expected_v_dot - 0]
  EXPECT_NEAR(error(0), expected_u_dot, 1e-6);
  EXPECT_NEAR(error(1), expected_v_dot, 1e-6);

  // Verify expected values
  EXPECT_NEAR(expected_u_dot, 0.0, 1e-6);
  EXPECT_NEAR(expected_v_dot, 60.0, 1e-6);
}

/**
 * @brief Test rotational flow only at image center
 *
 * Setup:
 * - Velocity: v_body = [0, 0, 0] m/s (no translation)
 * - Angular velocity: ω_body = [0.05, 0, 0] rad/s (roll rate in body frame)
 * - Altitude: h = 10 m
 * - Camera: nadir mount, focal length f = 600 px
 * - Image location: (x, y) = (0, 0) (image center)
 *
 * Expected flow at center:
 * - With nadir mount R_bc = [[0,1,0],[-1,0,0],[0,0,1]]:
 * - ω_cam = R_bc * [0.05,0,0] = [0, -0.05, 0] (pitch in camera frame)
 * - At center (x=0, y=0): rot_x = -ωy, rot_y = ωx
 * - u̇ = -(-0.05) = 0.05 rad/s → 30 px/s
 * - v̇ = 0 rad/s → 0 px/s
 */
TEST(OpticalFlowFactorTest, RotationOnly_ImageCenter) {
  // Setup
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  // State
  Pose3 pose = createTestPoseAGL(0, 0, h, 0);  // NED: h_agl = 10m
  Vector3 vel_world = Vector3::Zero();  // No translation (world frame)
  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());

  // Raw gyro measurement (body frame) - roll rate
  Vector3 gyro_meas(0.05, 0.0, 0.0);

  // Expected rotational flow in camera frame
  Vector3 omega_cam = R_bc * gyro_meas;  // [0, -0.05, 0]
  // At image center (x=0, y=0):
  // rot_x = xy·ωx - (1+x²)·ωy + y·ωz = 0 - ωy + 0 = -ωy
  // rot_y = (1+y²)·ωx - xy·ωy - x·ωz = ωx - 0 - 0 = ωx
  double expected_u_dot = focal_length * (-omega_cam.y());  // 30 px/s
  double expected_v_dot = focal_length * omega_cam.x();     // 0 px/s

  // Create factor with zero measured flow
  Vector2 measured_flow(0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Evaluate error with WORLD-FRAME velocity
  Vector error = factor.evaluateError(pose, vel_world, bias);

  // Debug output
  std::cout << "RotationOnly test:" << std::endl;
  std::cout << "  gyro_meas (body): [" << gyro_meas.transpose() << "]" << std::endl;
  std::cout << "  omega_cam expected: [" << omega_cam.transpose() << "]" << std::endl;
  std::cout << "  expected_u_dot: " << expected_u_dot << " px/s" << std::endl;
  std::cout << "  expected_v_dot: " << expected_v_dot << " px/s" << std::endl;
  std::cout << "  error(0): " << error(0) << " (predicted_u - measured_u)" << std::endl;
  std::cout << "  error(1): " << error(1) << " (predicted_v - measured_v)" << std::endl;

  // Verify
  EXPECT_NEAR(error(0), expected_u_dot, 1e-6);
  EXPECT_NEAR(error(1), expected_v_dot, 1e-6);

  EXPECT_NEAR(expected_u_dot, 30.0, 1e-6);
  EXPECT_NEAR(expected_v_dot, 0.0, 1e-6);
}

/**
 * @brief Test combined translation + rotation at image center
 *
 * Verifies superposition principle: total flow = translational + rotational
 */
TEST(OpticalFlowFactorTest, CombinedFlow_ImageCenter) {
  // Setup
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  // State with both velocity and rotation
  Pose3 pose = createTestPoseAGL(0, 0, h, 0);  // NED: h_agl = 10m
  Vector3 vel_body(1.0, 0.0, 0.0);  // 1 m/s forward in body

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());
  Vector3 gyro_meas(0.05, 0.0, 0.0);  // 0.05 rad/s roll

  // Expected components
  Vector3 v_cam = R_bc * vel_body;      // [0, -1, 0]
  Vector3 omega_cam = R_bc * gyro_meas; // [0, -0.05, 0]

  // Translational flow at center
  double trans_u = -focal_length * v_cam.x() / h;  // 0
  double trans_v = -focal_length * v_cam.y() / h;  // 60

  // Rotational flow at center
  double rot_u = focal_length * (-omega_cam.y());  // 30
  double rot_v = focal_length * omega_cam.x();     // 0

  // Total expected flow (superposition)
  double expected_u_dot = trans_u + rot_u;  // 30 px/s
  double expected_v_dot = trans_v + rot_v;  // 60 px/s

  // Create factor
  Vector2 measured_flow(0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Evaluate with WORLD-FRAME velocity
  Vector error = factor.evaluateError(pose, vel_world, bias);

  // Verify superposition
  EXPECT_NEAR(error(0), expected_u_dot, 1e-6);
  EXPECT_NEAR(error(1), expected_v_dot, 1e-6);
  EXPECT_NEAR(expected_u_dot, 30.0, 1e-6);
  EXPECT_NEAR(expected_v_dot, 60.0, 1e-6);
}


// ===== TEST SUITE 2: Gyro De-rotation =====

/**
 * @brief Test gyro de-biasing with zero bias
 */
TEST(OpticalFlowFactorTest, GyroDeBiasing_ZeroBias) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  Pose3 pose = createTestPose(0, 0, h, 0);
  Vector3 vel_world = Vector3::Zero();  // WORLD-FRAME (zero)

  // Zero bias → de-biased gyro should equal raw gyro
  Vector3 gyro_meas(0.05, 0.0, 0.0);
  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());

  Vector2 measured_flow(0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Should use full gyro measurement (no de-biasing)
  Vector error = factor.evaluateError(pose, vel_world, bias);

  // Expected: ω_cam = R_bc * gyro_meas = R_bc * [0.05, 0, 0]
  Vector3 omega_cam_expected = R_bc * gyro_meas;
  double expected_v_dot = focal_length * (omega_cam_expected.y() * 0.0);  // rot_y at center

  // This test mainly verifies no crash and reasonable output
  EXPECT_TRUE(std::isfinite(error(0)));
  EXPECT_TRUE(std::isfinite(error(1)));
}

/**
 * @brief Test gyro de-biasing with non-zero bias
 *
 * Verify that factor correctly subtracts bias from raw gyro
 */
TEST(OpticalFlowFactorTest, GyroDeBiasing_NonZeroBias) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  Pose3 pose = createTestPoseAGL(0, 0, h, 0);  // NED: h_agl = 10m
  Vector3 vel_world = Vector3::Zero();  // WORLD-FRAME (zero)

  // Raw gyro with bias
  Vector3 gyro_meas(0.05, 0.01, 0.0);
  Vector3 gyro_bias(0.0, 0.01, 0.0);  // Bias in Y-axis
  imuBias::ConstantBias bias = createTestBias(gyro_bias, Vector3::Zero());

  // Expected de-biased gyro
  Vector3 gyro_debiased = gyro_meas - gyro_bias;  // [0.05, 0, 0]

  Vector2 measured_flow(0.0, 0.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  Vector error = factor.evaluateError(pose, vel_world, bias);

  // De-biased gyro should be [0.05, 0, 0], so rotational flow should be non-zero
  EXPECT_TRUE(std::isfinite(error(0)));
  EXPECT_TRUE(std::isfinite(error(1)));

  // Verify that changing bias changes the error (proves de-biasing is happening)
  imuBias::ConstantBias bias_different = createTestBias(Vector3::Zero(), Vector3::Zero());
  Vector error_different = factor.evaluateError(pose, vel_world, bias_different);

  // Errors should be different because de-biasing is different
  EXPECT_GT(std::abs(error(0) - error_different(0)) + std::abs(error(1) - error_different(1)), 1e-6);
}

/**
 * @brief Verify imuBias::ConstantBias ordering: [accel(3), gyro(3)]
 */
TEST(OpticalFlowFactorTest, BiasOrdering) {
  Vector3 accel_bias(0.1, 0.2, 0.3);
  Vector3 gyro_bias(0.01, 0.02, 0.03);

  imuBias::ConstantBias bias(accel_bias, gyro_bias);

  // Verify extraction
  EXPECT_NEAR(bias.accelerometer()(0), 0.1, 1e-9);
  EXPECT_NEAR(bias.accelerometer()(1), 0.2, 1e-9);
  EXPECT_NEAR(bias.accelerometer()(2), 0.3, 1e-9);

  EXPECT_NEAR(bias.gyroscope()(0), 0.01, 1e-9);
  EXPECT_NEAR(bias.gyroscope()(1), 0.02, 1e-9);
  EXPECT_NEAR(bias.gyroscope()(2), 0.03, 1e-9);
}


// ===== TEST SUITE 3: Camera Transformations =====

/**
 * @brief Test body→camera velocity transformation
 */
TEST(OpticalFlowFactorTest, BodyToCameraVelocity) {
  Matrix3 R_bc = nadirMountRotation();

  // Body frame: X-forward, Y-left, Z-up
  // Camera frame: X-right, Y-down, Z-forward

  // Test 1: Forward velocity in body → Z-forward in camera
  Vector3 vel_body_forward(1.0, 0.0, 0.0);
  Vector3 vel_cam_forward = R_bc * vel_body_forward;
  EXPECT_NEAR(vel_cam_forward(0), 0.0, 1e-9);   // X_cam
  EXPECT_NEAR(vel_cam_forward(1), -1.0, 1e-9);  // Y_cam (down)
  EXPECT_NEAR(vel_cam_forward(2), 0.0, 1e-9);   // Z_cam

  // Test 2: Left velocity in body → -X in camera (right is positive)
  Vector3 vel_body_left(0.0, 1.0, 0.0);
  Vector3 vel_cam_left = R_bc * vel_body_left;
  EXPECT_NEAR(vel_cam_left(0), 1.0, 1e-9);   // X_cam (right)
  EXPECT_NEAR(vel_cam_left(1), 0.0, 1e-9);   // Y_cam
  EXPECT_NEAR(vel_cam_left(2), 0.0, 1e-9);   // Z_cam

  // Test 3: Up velocity in body → Z in camera
  Vector3 vel_body_up(0.0, 0.0, 1.0);
  Vector3 vel_cam_up = R_bc * vel_body_up;
  EXPECT_NEAR(vel_cam_up(0), 0.0, 1e-9);  // X_cam
  EXPECT_NEAR(vel_cam_up(1), 0.0, 1e-9);  // Y_cam
  EXPECT_NEAR(vel_cam_up(2), 1.0, 1e-9);  // Z_cam (forward)
}

/**
 * @brief Test nadir mount rotation matrix
 */
TEST(OpticalFlowFactorTest, NadirMountMatrix) {
  Matrix3 R_bc = nadirMountRotation();

  // Verify matrix structure
  EXPECT_NEAR(R_bc(0, 0), 0.0, 1e-9);
  EXPECT_NEAR(R_bc(0, 1), 1.0, 1e-9);
  EXPECT_NEAR(R_bc(0, 2), 0.0, 1e-9);

  EXPECT_NEAR(R_bc(1, 0), -1.0, 1e-9);
  EXPECT_NEAR(R_bc(1, 1), 0.0, 1e-9);
  EXPECT_NEAR(R_bc(1, 2), 0.0, 1e-9);

  EXPECT_NEAR(R_bc(2, 0), 0.0, 1e-9);
  EXPECT_NEAR(R_bc(2, 1), 0.0, 1e-9);
  EXPECT_NEAR(R_bc(2, 2), 1.0, 1e-9);

  // Verify it's a valid rotation (orthonormal)
  Matrix3 identity = R_bc * R_bc.transpose();
  EXPECT_NEAR(identity(0, 0), 1.0, 1e-9);
  EXPECT_NEAR(identity(1, 1), 1.0, 1e-9);
  EXPECT_NEAR(identity(2, 2), 1.0, 1e-9);
  EXPECT_NEAR(identity(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(identity(0, 2), 0.0, 1e-9);
  EXPECT_NEAR(identity(1, 2), 0.0, 1e-9);

  // Verify determinant = +1
  EXPECT_NEAR(R_bc.determinant(), 1.0, 1e-9);
}


// ===== TEST SUITE 4: Jacobian Accuracy =====

/**
 * @brief Test velocity Jacobian against numerical derivative
 */
TEST(OpticalFlowFactorTest, VelocityJacobian_Numerical) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  Pose3 pose = createTestPose(0, 0, h, 0);
  Vector3 vel_body(1.0, 0.5, 0.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());
  Vector3 gyro_meas(0.0, 0.0, 0.05);

  Vector2 measured_flow(10.0, 20.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Get analytical Jacobian
  Matrix H_vel_analytical;
  factor.evaluateError(pose, vel_world, bias, boost::none, H_vel_analytical, boost::none);

  // Compute numerical Jacobian
  auto error_func = [&](const Vector& v) -> Vector {
    Vector3 vel(v(0), v(1), v(2));
    return factor.evaluateError(pose, vel, bias);
  };
  Matrix H_vel_numerical = numericalDerivative(error_func, vel_world);

  // Compare
  EXPECT_EQ(H_vel_analytical.rows(), 2);
  EXPECT_EQ(H_vel_analytical.cols(), 3);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(H_vel_analytical(i, j), H_vel_numerical(i, j), 1e-4)
          << "Mismatch at (" << i << ", " << j << ")";
    }
  }
}

/**
 * @brief Test pose Jacobian against numerical derivative
 */
TEST(OpticalFlowFactorTest, PoseJacobian_Numerical) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  Pose3 pose = createTestPose(0, 0, h, 0);
  Vector3 vel_body(1.0, 0.5, 0.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());
  Vector3 gyro_meas(0.0, 0.0, 0.05);

  Vector2 measured_flow(10.0, 20.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Get analytical Jacobian
  Matrix H_pose_analytical;
  factor.evaluateError(pose, vel_world, bias, H_pose_analytical, boost::none, boost::none);

  // Compute numerical Jacobian (using Pose3::Logmap for tangent space)
  auto error_func = [&](const Vector& xi) -> Vector {
    Pose3 p = pose.retract(xi);
    // Need to recompute vel_world for the new pose
    Vector3 vel_w = p.rotation().matrix() * vel_body;
    return factor.evaluateError(p, vel_w, bias);
  };
  Vector6 zero = Vector6::Zero();
  Matrix H_pose_numerical = numericalDerivative(error_func, zero);

  // Compare
  EXPECT_EQ(H_pose_analytical.rows(), 2);
  EXPECT_EQ(H_pose_analytical.cols(), 6);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(H_pose_analytical(i, j), H_pose_numerical(i, j), 1e-4)
          << "Mismatch at (" << i << ", " << j << ")";
    }
  }
}

/**
 * @brief Test bias Jacobian against numerical derivative
 */
TEST(OpticalFlowFactorTest, BiasJacobian_Numerical) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  Pose3 pose = createTestPose(0, 0, h, 0);
  Vector3 vel_body(1.0, 0.5, 0.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  Vector3 gyro_bias_init(0.01, 0.01, 0.01);
  imuBias::ConstantBias bias = createTestBias(gyro_bias_init, Vector3::Zero());
  Vector3 gyro_meas(0.0, 0.0, 0.05);

  Vector2 measured_flow(10.0, 20.0);
  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  // Get analytical Jacobian
  Matrix H_bias_analytical;
  factor.evaluateError(pose, vel_world, bias, boost::none, boost::none, H_bias_analytical);

  // Compute numerical Jacobian (using ConstantBias tangent space)
  auto error_func = [&](const Vector& db) -> Vector {
    imuBias::ConstantBias b = bias.retract(db);
    return factor.evaluateError(pose, vel_world, b);
  };
  Vector6 zero = Vector6::Zero();
  Matrix H_bias_numerical = numericalDerivative(error_func, zero);

  // Compare
  EXPECT_EQ(H_bias_analytical.rows(), 2);
  EXPECT_EQ(H_bias_analytical.cols(), 6);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(H_bias_analytical(i, j), H_bias_numerical(i, j), 1e-4)
          << "Mismatch at (" << i << ", " << j << ")";
    }
  }
}


// ===== TEST SUITE 5: Integration Tests =====

/**
 * @brief Test factor in minimal optimization
 */
TEST(OpticalFlowFactorTest, FactorInGraph_Optimization) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  // Ground truth state
  Pose3 pose_gt = createTestPose(0, 0, h, 0);
  Vector3 vel_body_gt(1.0, 0.0, 0.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world_gt = pose_gt.rotation().matrix() * vel_body_gt;

  imuBias::ConstantBias bias_gt = createTestBias(Vector3::Zero(), Vector3::Zero());
  Vector3 gyro_meas(0.0, 0.0, 0.05);

  // Compute ground truth flow
  Vector3 v_cam = R_bc * vel_body_gt;
  Vector3 omega_cam = R_bc * gyro_meas;
  double gt_u_dot = focal_length * (-v_cam.x() / h - omega_cam.y());
  double gt_v_dot = focal_length * (-v_cam.y() / h + omega_cam.x());
  Vector2 measured_flow(gt_u_dot, gt_v_dot);

  // Create graph with priors
  NonlinearFactorGraph graph;
  auto prior_noise_pose = noiseModel::Isotropic::Sigma(6, 0.1);
  auto prior_noise_vel = noiseModel::Isotropic::Sigma(3, 0.1);
  auto prior_noise_bias = noiseModel::Isotropic::Sigma(6, 0.01);

  graph.addPrior(X(0), pose_gt, prior_noise_pose);
  graph.addPrior(V(0), vel_world_gt, prior_noise_vel);
  graph.addPrior(B(0), bias_gt, prior_noise_bias);

  // Add optical flow factor
  auto flow_noise = noiseModel::Isotropic::Sigma(2, 1.0);
  auto of_factor = boost::make_shared<OpticalFlowFactor>(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      flow_noise);
  graph.push_back(of_factor);

  // Initial values (perturbed)
  Values initial;
  Vector6 pose_pert = Vector6::Random() * 0.01;
  Vector3 vel_pert = Vector3::Random() * 0.01;
  Vector3 vel_init = vel_world_gt + vel_pert;  // Force evaluation
  initial.insert(X(0), pose_gt.retract(pose_pert));
  initial.insert(V(0), vel_init);
  initial.insert(B(0), bias_gt);

  // Optimize
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Verify convergence to ground truth
  Pose3 pose_opt = result.at<Pose3>(X(0));
  Vector3 vel_opt = result.at<Vector3>(V(0));

  EXPECT_LT(pose_opt.translation().norm() - pose_gt.translation().norm(), 0.1);
  EXPECT_LT((vel_opt - vel_world_gt).norm(), 0.1);
}

/**
 * @brief Test that residuals are reasonable
 */
TEST(OpticalFlowFactorTest, ResidualMagnitudes) {
  const double h = 10.0;
  const double focal_length = 600.0;
  Matrix3 R_bc = nadirMountRotation();

  Pose3 pose = createTestPose(0, 0, h, 0);
  Vector3 vel_body(1.0, 0.5, 0.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  imuBias::ConstantBias bias = createTestBias(Vector3::Zero(), Vector3::Zero());
  Vector3 gyro_meas(0.0, 0.0, 0.05);

  // Perfect measurement (zero error)
  Vector3 v_cam = R_bc * vel_body;
  Vector3 omega_cam = R_bc * gyro_meas;
  double u_dot = focal_length * (-v_cam.x() / h - omega_cam.y());
  double v_dot = focal_length * (-v_cam.y() / h + omega_cam.x());
  Vector2 measured_flow(u_dot, v_dot);

  auto noise_model = noiseModel::Isotropic::Sigma(2, 1.0);
  OpticalFlowFactor factor(
      X(0), V(0), B(0),
      measured_flow,
      gyro_meas,
      focal_length,
      R_bc,
      noise_model);

  Vector error = factor.evaluateError(pose, vel_world, bias);

  // Error should be near zero (perfect measurement)
  EXPECT_NEAR(error.norm(), 0.0, 1e-6);
}


// ===== TEST SUITE 6: AeroVelocityFactor Tests =====

/**
 * @brief Test AeroVelocityFactor basic error evaluation
 */
TEST(AeroVelocityFactorTest, BasicError) {
  // Setup
  Pose3 pose = createTestPose(0, 0, 10, 0);  // Zero yaw → body = world
  Vector3 vel_body(5.0, 0.0, 0.0);  // 5 m/s forward in body

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  // Measured airspeed matches ground truth
  const double measured_airspeed = vel_body.norm();  // 5.0 m/s

  auto noise_model = noiseModel::Isotropic::Sigma(1, 0.3);
  AeroVelocityFactor factor(X(0), V(0), measured_airspeed, noise_model);

  // Evaluate error (should be zero for perfect measurement)
  Vector error = factor.evaluateError(pose, vel_world);

  EXPECT_NEAR(error(0), 0.0, 1e-6);
}

/**
 * @brief Test AeroVelocityFactor with non-zero error
 */
TEST(AeroVelocityFactorTest, NonZeroError) {
  // Setup
  Pose3 pose = createTestPose(0, 0, 10, 0);
  Vector3 vel_body(5.0, 3.0, 0.0);  // Actual velocity: sqrt(25+9) = sqrt(34) ≈ 5.83 m/s

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  // Measured airspeed is different
  const double measured_airspeed = 5.0;  // m/s
  const double actual_airspeed = vel_body.norm();  // 5.83 m/s

  auto noise_model = noiseModel::Isotropic::Sigma(1, 0.3);
  AeroVelocityFactor factor(X(0), V(0), measured_airspeed, noise_model);

  // Evaluate error
  Vector error = factor.evaluateError(pose, vel_world);

  // Error should be: predicted - measured = actual - measured
  const double expected_error = actual_airspeed - measured_airspeed;
  EXPECT_NEAR(error(0), expected_error, 1e-6);
  EXPECT_NEAR(expected_error, 0.83, 0.01);
}

/**
 * @brief Test AeroVelocityFactor with rotated pose (non-zero yaw)
 */
TEST(AeroVelocityFactorTest, RotatedPose) {
  // Setup with 45-degree yaw
  const double yaw = M_PI / 4.0;  // 45 degrees
  Pose3 pose = createTestPose(0, 0, 10, yaw);

  // Body-frame velocity
  Vector3 vel_body(5.0, 0.0, 0.0);  // 5 m/s forward in body

  // WORLD-FRAME VELOCITY: rotated by yaw
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  // Airspeed magnitude should be independent of orientation
  const double measured_airspeed = 5.0;  // m/s

  auto noise_model = noiseModel::Isotropic::Sigma(1, 0.3);
  AeroVelocityFactor factor(X(0), V(0), measured_airspeed, noise_model);

  // Evaluate error
  Vector error = factor.evaluateError(pose, vel_world);

  // Error should still be zero (airspeed is orientation-independent)
  EXPECT_NEAR(error(0), 0.0, 1e-6);
}

/**
 * @brief Test AeroVelocityFactor velocity Jacobian
 */
TEST(AeroVelocityFactorTest, VelocityJacobian) {
  // Setup
  Pose3 pose = createTestPose(0, 0, 10, 0);
  Vector3 vel_body(5.0, 3.0, 1.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  const double measured_airspeed = 5.0;

  auto noise_model = noiseModel::Isotropic::Sigma(1, 0.3);
  AeroVelocityFactor factor(X(0), V(0), measured_airspeed, noise_model);

  // Get analytical Jacobian
  Matrix H_vel_analytical;
  factor.evaluateError(pose, vel_world, boost::none, H_vel_analytical);

  // Compute numerical Jacobian
  auto error_func = [&](const Vector& v) -> Vector {
    Vector3 vel(v(0), v(1), v(2));
    return factor.evaluateError(pose, vel);
  };
  Matrix H_vel_numerical = numericalDerivative(error_func, vel_world);

  // Compare
  EXPECT_EQ(H_vel_analytical.rows(), 1);
  EXPECT_EQ(H_vel_analytical.cols(), 3);

  for (int j = 0; j < 3; ++j) {
    EXPECT_NEAR(H_vel_analytical(0, j), H_vel_numerical(0, j), 1e-4)
        << "Velocity Jacobian mismatch at column " << j;
  }
}

/**
 * @brief Test AeroVelocityFactor pose Jacobian
 */
TEST(AeroVelocityFactorTest, PoseJacobian) {
  // Setup with non-zero yaw to make rotation affect velocity transformation
  const double yaw = M_PI / 6.0;  // 30 degrees
  Pose3 pose = createTestPose(0, 0, 10, yaw);
  Vector3 vel_body(5.0, 3.0, 0.0);

  // WORLD-FRAME VELOCITY
  Vector3 vel_world = pose.rotation().matrix() * vel_body;

  const double measured_airspeed = 5.0;

  auto noise_model = noiseModel::Isotropic::Sigma(1, 0.3);
  AeroVelocityFactor factor(X(0), V(0), measured_airspeed, noise_model);

  // Get analytical Jacobian
  Matrix H_pose_analytical;
  factor.evaluateError(pose, vel_world, H_pose_analytical, boost::none);

  // Compute numerical Jacobian
  auto error_func = [&](const Vector& xi) -> Vector {
    Pose3 p = pose.retract(xi);
    // Recompute vel_world for new pose
    Vector3 vel_w = p.rotation().matrix() * vel_body;
    return factor.evaluateError(p, vel_w);
  };
  Vector6 zero = Vector6::Zero();
  Matrix H_pose_numerical = numericalDerivative(error_func, zero);

  // Compare
  EXPECT_EQ(H_pose_analytical.rows(), 1);
  EXPECT_EQ(H_pose_analytical.cols(), 6);

  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(H_pose_analytical(0, j), H_pose_numerical(0, j), 1e-4)
        << "Pose Jacobian mismatch at column " << j;
  }
}

/**
 * @brief Test AeroVelocityFactor in optimization
 */
TEST(AeroVelocityFactorTest, FactorInGraph_Optimization) {
  // Ground truth
  Pose3 pose_gt = createTestPose(0, 0, 10, 0);
  Vector3 vel_body_gt(6.0, 0.0, 0.0);
  Vector3 vel_world_gt = pose_gt.rotation().matrix() * vel_body_gt;
  const double measured_airspeed = 6.0;  // m/s

  // Create graph with priors
  NonlinearFactorGraph graph;
  auto prior_noise_pose = noiseModel::Isotropic::Sigma(6, 0.1);
  auto prior_noise_vel = noiseModel::Isotropic::Sigma(3, 1.0);  // Loose prior on velocity

  graph.addPrior(X(0), pose_gt, prior_noise_pose);
  Vector3 zero_vel(0.0, 0.0, 0.0);
  graph.addPrior(V(0), zero_vel, prior_noise_vel);  // Wrong initial guess

  // Add airspeed factor (should pull velocity toward correct magnitude)
  auto airspeed_noise = noiseModel::Isotropic::Sigma(1, 0.3);
  auto aero_factor = boost::make_shared<AeroVelocityFactor>(
      X(0), V(0), measured_airspeed, airspeed_noise);
  graph.push_back(aero_factor);

  // Initial values
  Values initial;
  initial.insert(X(0), pose_gt);
  initial.insert(V(0), Vector3(3.0, 0.0, 0.0));  // Wrong magnitude

  // Optimize
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  Values result = optimizer.optimize();

  // Check that velocity magnitude is close to measured airspeed
  Vector3 vel_opt = result.at<Vector3>(V(0));
  Pose3 pose_opt = result.at<Pose3>(X(0));

  // Transform to body frame to get airspeed
  Vector3 vel_body_opt = pose_opt.rotation().matrix().transpose() * vel_opt;
  double airspeed_opt = vel_body_opt.norm();

  EXPECT_NEAR(airspeed_opt, measured_airspeed, 0.5);  // Should be close
}

/**
 * @brief Test zero-velocity edge case
 */
TEST(AeroVelocityFactorTest, ZeroVelocity) {
  // Setup with zero velocity
  Pose3 pose = createTestPose(0, 0, 10, 0);
  Vector3 vel_world = Vector3::Zero();
  const double measured_airspeed = 0.1;  // Small measurement

  auto noise_model = noiseModel::Isotropic::Sigma(1, 0.3);
  AeroVelocityFactor factor(X(0), V(0), measured_airspeed, noise_model);

  // Evaluate (should handle zero velocity gracefully)
  Matrix H_pose, H_vel;
  Vector error = factor.evaluateError(pose, vel_world, H_pose, H_vel);

  // Should not crash and Jacobians should be zero
  EXPECT_TRUE(std::isfinite(error(0)));
  EXPECT_EQ(H_pose.rows(), 1);
  EXPECT_EQ(H_pose.cols(), 6);
  EXPECT_EQ(H_vel.rows(), 1);
  EXPECT_EQ(H_vel.cols(), 3);

  // All Jacobian entries should be zero (or very small)
  EXPECT_LT(H_pose.norm(), 1e-3);
  EXPECT_LT(H_vel.norm(), 1e-3);
}


// ===== Main =====

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
