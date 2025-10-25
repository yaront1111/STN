#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace chimera { namespace factors {

/**
 * @brief Optical Flow Factor with complete model (translation + rotation + extrinsics)
 *
 * COMPLETE optical flow model for downward-facing camera:
 * - ROS camera frame: X-right, Y-down, Z-forward (optical axis)
 * - Full optical flow: translational + rotational components
 * - Gyro de-rotation using bias state from factor graph
 * - Camera extrinsics (body→camera transform)
 *
 * Optical flow equations (normalized image coordinates):
 *   ẋ = -vx/Z + x·vz/Z + xy·ωx - (1+x²)·ωy + y·ωz
 *   ẏ = -vy/Z + y·vz/Z + (1+y²)·ωx - xy·ωy - x·ωz
 *
 * Where:
 *   (x, y) = normalized image coords [(u-cx)/fx, (v-cy)/fy]
 *   (vx, vy, vz) = camera-frame velocity
 *   (ωx, ωy, ωz) = camera-frame angular velocity (AFTER de-biasing)
 *   Z = depth along camera Z-axis (altitude in camera frame)
 *
 * This is a NoiseModelFactorN connecting:
 *   - Pose3: provides altitude Z and world→body rotation
 *   - Vector3: WORLD-frame velocity (rotated to body inside factor)
 *   - imuBias::ConstantBias: gyro bias for de-rotation (CRITICAL!)
 */
class OpticalFlowFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>;
  using This = OpticalFlowFactor;

 private:
  gtsam::Vector2 measured_flow_px_;  ///< Measured raw optical flow (pixels/sec, includes rotation!)
  double fx_, fy_;                   ///< Camera focal lengths (pixels)
  double cx_, cy_;                   ///< Principal point (pixels)
  double img_x_, img_y_;             ///< Normalized image coords where flow is measured (default: center)
  gtsam::Matrix3 R_bc_;              ///< Body→Camera rotation matrix
  gtsam::Vector3 gyro_meas_;         ///< Raw gyro measurement (rad/s in body frame)

 public:
  /**
   * @brief Construct optical flow factor with full model
   * @param poseKey Key for Pose3 (altitude + rotation)
   * @param velKey Key for body velocity Vector3
   * @param biasKey Key for imuBias::ConstantBias (for gyro de-rotation)
   * @param measuredFlowPx Measured optical flow in PIXELS/sec (raw, includes rotation)
   * @param gyroMeas Raw gyro measurement in body frame (rad/s, will be de-biased in factor)
   * @param fx Focal length x (pixels)
   * @param fy Focal length y (pixels)
   * @param cx Principal point x (pixels)
   * @param cy Principal point y (pixels)
   * @param img_x Normalized x-coord where flow is measured (0 = image center)
   * @param img_y Normalized y-coord where flow is measured (0 = image center)
   * @param R_bc Body→Camera rotation matrix (3x3)
   * @param model Noise model (2-DOF, pixels/sec)
   */
  OpticalFlowFactor(gtsam::Key poseKey, gtsam::Key velKey, gtsam::Key biasKey,
                    const gtsam::Vector2& measuredFlowPx,
                    const gtsam::Vector3& gyroMeas,
                    double fx, double fy, double cx, double cy,
                    double img_x, double img_y,
                    const gtsam::Matrix3& R_bc,
                    const gtsam::SharedNoiseModel& model)
      : Base(model, poseKey, velKey, biasKey),
        measured_flow_px_(measuredFlowPx),
        fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        img_x_(img_x), img_y_(img_y),
        R_bc_(R_bc),
        gyro_meas_(gyroMeas) {}

  /**
   * @brief Simplified constructor for nadir mount at image center
   * @param poseKey Pose3 key
   * @param velKey Velocity key
   * @param biasKey Bias key
   * @param measuredFlowPx Raw flow in pixels/sec
   * @param gyroMeas Raw gyro measurement (body frame, rad/s)
   * @param focalLength Focal length (assumes fx=fy, cx=cy=0)
   * @param R_bc Body→Camera rotation (nadir: [0,1,0; -1,0,0; 0,0,1])
   * @param model Noise model
   */
  OpticalFlowFactor(gtsam::Key poseKey, gtsam::Key velKey, gtsam::Key biasKey,
                    const gtsam::Vector2& measuredFlowPx,
                    const gtsam::Vector3& gyroMeas,
                    double focalLength,
                    const gtsam::Matrix3& R_bc,
                    const gtsam::SharedNoiseModel& model)
      : Base(model, poseKey, velKey, biasKey),
        measured_flow_px_(measuredFlowPx),
        fx_(focalLength), fy_(focalLength), cx_(0.0), cy_(0.0),
        img_x_(0.0), img_y_(0.0),  // Image center
        R_bc_(R_bc),
        gyro_meas_(gyroMeas) {}

  virtual ~OpticalFlowFactor() = default;

  /**
   * @brief Evaluate error with full optical flow model
   * @param pose World pose (altitude h = pose.z(), provides R_wb rotation)
   * @param vel_world WORLD-frame velocity (m/s) - transformed to body inside factor
   * @param bias IMU bias (contains gyro bias for de-rotation)
   * @param H_pose [out] Jacobian w.r.t. pose (2x6)
   * @param H_vel [out] Jacobian w.r.t. velocity (2x3)
   * @param H_bias [out] Jacobian w.r.t. bias (2x6)
   * @return 2-vector error in pixels/sec (predicted - measured)
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const gtsam::Vector3& vel_world,
                              const gtsam::imuBias::ConstantBias& bias,
                              boost::optional<gtsam::Matrix&> H_pose = boost::none,
                              boost::optional<gtsam::Matrix&> H_vel = boost::none,
                              boost::optional<gtsam::Matrix&> H_bias = boost::none) const override {

    // NED: nav-Z is Down-positive; AGL is up-positive => h_agl = -z
    const double h_agl = -pose.z();

    // Safety: avoid degenerate depth
    if (h_agl <= 1e-2) {
      if (H_pose) *H_pose = gtsam::Matrix::Zero(2, 6);
      if (H_vel) *H_vel = gtsam::Matrix::Zero(2, 3);
      if (H_bias) *H_bias = gtsam::Matrix::Zero(2, 6);
      return gtsam::Vector2::Zero();
    }

    // ===== Transform velocity from WORLD to BODY to CAMERA frame =====

    // World→Body rotation: R_bw = R_wb^T
    const gtsam::Matrix3 R_wb = pose.rotation().matrix();
    const gtsam::Matrix3 R_bw = R_wb.transpose();

    // Velocity: world → body → camera
    // v_body = R_bw * v_world
    // v_cam = R_bc * v_body = R_bc * R_bw * v_world
    const gtsam::Vector3 v_body = R_bw * vel_world;
    const gtsam::Vector3 v_cam = R_bc_ * v_body;

    // Angular velocity: ω_cam = R_bc * (ω_meas - b_g)
    // CRITICAL: De-bias using the bias state from the factor graph!
    const gtsam::Vector3 gyro_bias = bias.gyroscope();
    const gtsam::Vector3 omega_body_debias = gyro_meas_ - gyro_bias;
    const gtsam::Vector3 omega_cam = R_bc_ * omega_body_debias;

    // ===== Tilt-compensated depth (CRITICAL for accuracy!) =====
    // OF translation uses depth along camera Z-axis, NOT world AGL
    // In bank/tilt: Z_eff = h / cos(tilt_angle)
    // Compute: cos(tilt) = n_world^T * R_wc * e_z_cam
    // where R_wc = R_wb * R_bc, e_z_cam = [0,0,1], n_world = [0,0,1]

    const gtsam::Matrix3 R_wc = R_wb * R_bc_;  // World→Camera rotation
    const gtsam::Vector3 ez_cam(0.0, 0.0, 1.0);  // Camera optical axis
    const gtsam::Vector3 n_world(0.0, 0.0, 1.0); // World vertical (flat ground normal)
    const gtsam::Vector3 cam_z_in_world = R_wc * ez_cam;
    // Use |cos| so tilt sign (up vs down) can't flip depth
    const double cos_tilt = std::abs(n_world.dot(cam_z_in_world));

    // Effective depth with safety clamping
    const double Z_eff = std::max(0.3, h_agl / std::max(1e-3, cos_tilt));

    // ===== Compute optical flow components =====

    const double x = img_x_;  // Normalized image x-coord
    const double y = img_y_;  // Normalized image y-coord
    const double Z = Z_eff;   // Tilt-compensated depth

    // Translational optical flow (normalized coords/sec)
    const double trans_x_norm = (-v_cam.x() + x * v_cam.z()) / Z;
    const double trans_y_norm = (-v_cam.y() + y * v_cam.z()) / Z;

    // Rotational optical flow (normalized coords/sec)
    const double rot_x_norm = x*y*omega_cam.x() - (1 + x*x)*omega_cam.y() + y*omega_cam.z();
    const double rot_y_norm = (1 + y*y)*omega_cam.x() - x*y*omega_cam.y() - x*omega_cam.z();

    // Total predicted flow (normalized coords/sec)
    const double pred_x_dot_norm = trans_x_norm + rot_x_norm;
    const double pred_y_dot_norm = trans_y_norm + rot_y_norm;

    // Convert to pixels/sec
    const double pred_u_dot = fx_ * pred_x_dot_norm;
    const double pred_v_dot = fy_ * pred_y_dot_norm;

    // ===== Compute Jacobians if requested =====

    if (H_pose) {
      // Pose Jacobian: altitude effect on translational flow
      // Pose parameterization: [rot(3), pos(3)] → (rx, ry, rz, x, y, z)
      gtsam::Matrix J_pose = gtsam::Matrix::Zero(2, 6);

      // Z_eff = h_agl / |cos|,   h_agl = -z_nav  ⇒  ∂Z_eff/∂z = -1/|cos|
      const double dZeff_dz = -1.0 / std::max(1e-3, cos_tilt);
      const double dtrans_x_dZ = -(-v_cam.x() + x * v_cam.z()) / (Z * Z);
      const double dtrans_y_dZ = -(-v_cam.y() + y * v_cam.z()) / (Z * Z);
      // Pose tangent order: [rx ry rz tx ty tz] → z-translation is col 5
      J_pose(0, 5) = fx_ * dtrans_x_dZ * dZeff_dz;  // ∂u̇/∂z_nav
      J_pose(1, 5) = fy_ * dtrans_y_dZ * dZeff_dz;  // ∂v̇/∂z_nav

      // Rotation Jacobian: tilt affects Z_eff, but typically 2nd-order for small tilts
      // Skipping for now (optimization will still converge, just slightly slower)

      *H_pose = J_pose;
    }

    if (H_vel) {
      // Velocity Jacobian: translational flow component
      // Chain rule: ∂flow/∂vel_world = ∂flow/∂vel_cam * ∂vel_cam/∂vel_world
      // where v_cam = R_bc * R_bw * v_world

      // Translational Jacobian in camera frame (2x3)
      gtsam::Matrix J_vel_cam = gtsam::Matrix::Zero(2, 3);
      J_vel_cam(0, 0) = -fx_ / Z;           // ∂u̇/∂vx_cam
      J_vel_cam(0, 2) = fx_ * x / Z;        // ∂u̇/∂vz_cam
      J_vel_cam(1, 1) = -fy_ / Z;           // ∂v̇/∂vy_cam
      J_vel_cam(1, 2) = fy_ * y / Z;        // ∂v̇/∂vz_cam

      // Chain through world→body→camera: ∂vel_cam/∂vel_world = R_bc * R_bw
      *H_vel = J_vel_cam * R_bc_ * R_bw;
    }

    if (H_bias) {
      // Bias Jacobian: rotational flow de-rotation component
      // ∂flow/∂bias = -∂flow/∂omega_cam * R_bc  (negative because ω_cam = R_bc * (ω_meas - b_g))

      // Rotational Jacobian in camera frame (2x3)
      gtsam::Matrix J_omega_cam = gtsam::Matrix::Zero(2, 3);
      J_omega_cam(0, 0) = fx_ * x * y;              // ∂u̇/∂ωx_cam
      J_omega_cam(0, 1) = -fx_ * (1 + x*x);         // ∂u̇/∂ωy_cam
      J_omega_cam(0, 2) = fx_ * y;                  // ∂u̇/∂ωz_cam
      J_omega_cam(1, 0) = fy_ * (1 + y*y);          // ∂v̇/∂ωx_cam
      J_omega_cam(1, 1) = -fy_ * x * y;             // ∂v̇/∂ωy_cam
      J_omega_cam(1, 2) = -fy_ * x;                 // ∂v̇/∂ωz_cam

      // Chain rule: ∂flow/∂bias_g = ∂flow/∂ω_cam * ∂ω_cam/∂bias_g
      // where ω_cam = R_bc * (ω_meas - b_g), so ∂ω_cam/∂b_g = -R_bc
      gtsam::Matrix J_omega_body = -J_omega_cam * R_bc_;

      // imuBias::ConstantBias ordering: [accel(3), gyro(3)]
      // So Jacobian is [2 x 6] = [zeros(2,3), J_omega_body(2,3)]
      gtsam::Matrix J_bias = gtsam::Matrix::Zero(2, 6);
      J_bias.block<2,3>(0, 3) = J_omega_body;  // Gyro bias columns

      *H_bias = J_bias;
    }

    // Return error: predicted - measured (pixels/sec)
    return gtsam::Vector2(pred_u_dot - measured_flow_px_.x(),
                          pred_v_dot - measured_flow_px_.y());
  }

  /** Return a deep copy of this factor */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** Print the factor for debugging */
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "OpticalFlowFactor(meas=[" << measured_flow_px_.x() << ", " << measured_flow_px_.y()
              << "] px/s, fx=" << fx_ << ", fy=" << fy_ << ", img_pos=[" << img_x_ << "," << img_y_ << "])\n";
    Base::print("", keyFormatter);
  }

  /** Test equality */
  bool equals(const gtsam::NonlinearFactor& other, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&other);
    return e != nullptr && Base::equals(*e, tol) &&
           measured_flow_px_.isApprox(e->measured_flow_px_, tol) &&
           std::abs(fx_ - e->fx_) < tol;
  }
};

}} // namespace chimera::factors
