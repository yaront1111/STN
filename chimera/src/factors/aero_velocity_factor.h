#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace chimera { namespace factors {

/**
 * @brief Aerodynamic velocity (airspeed) factor
 *
 * Models airspeed sensor measurement:
 *   measured_airspeed ≈ ||v_body|| = ||R_bw * v_world||
 *
 * This is a scalar constraint (1-DOF) on velocity magnitude in the body frame.
 * Provides scale observability for velocity estimation (critical for OF).
 *
 * Frame transformation:
 *   v_body = R_bw * v_world, where R_bw = R_wb^T (from pose)
 *
 * This is a NoiseModelFactor2 connecting:
 *   - Pose3: provides world→body rotation R_wb
 *   - Vector3: WORLD-frame velocity (transformed to body inside factor)
 */
class AeroVelocityFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
 public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>;
  using This = AeroVelocityFactor;

 private:
  double measured_airspeed_;  ///< Measured airspeed (m/s, scalar)

 public:
  /**
   * @brief Construct airspeed factor
   * @param poseKey Key for Pose3 (provides R_wb rotation)
   * @param velKey Key for world-frame velocity Vector3
   * @param measuredAirspeed Measured airspeed (m/s, scalar magnitude)
   * @param model Noise model (1-DOF, m/s)
   */
  AeroVelocityFactor(gtsam::Key poseKey, gtsam::Key velKey,
                     double measuredAirspeed,
                     const gtsam::SharedNoiseModel& model)
      : Base(model, poseKey, velKey),
        measured_airspeed_(measuredAirspeed) {}

  virtual ~AeroVelocityFactor() = default;

  /**
   * @brief Evaluate error: predicted airspeed - measured airspeed
   * @param pose World pose (provides R_wb rotation)
   * @param vel_world WORLD-frame velocity (m/s)
   * @param H_pose [out] Jacobian w.r.t. pose (1x6)
   * @param H_vel [out] Jacobian w.r.t. velocity (1x3)
   * @return 1-vector error (m/s)
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const gtsam::Vector3& vel_world,
                              boost::optional<gtsam::Matrix&> H_pose = boost::none,
                              boost::optional<gtsam::Matrix&> H_vel = boost::none) const override {
    // Transform velocity: world → body
    const gtsam::Matrix3 R_wb = pose.rotation().matrix();
    const gtsam::Matrix3 R_bw = R_wb.transpose();
    const gtsam::Vector3 v_body = R_bw * vel_world;

    // Predicted airspeed (magnitude)
    const double v_body_norm = v_body.norm();

    // Safety: avoid division by zero in Jacobian
    if (v_body_norm < 1e-6) {
      if (H_pose) *H_pose = gtsam::Matrix::Zero(1, 6);
      if (H_vel) *H_vel = gtsam::Matrix::Zero(1, 3);
      return gtsam::Vector1(0.0 - measured_airspeed_);
    }

    // ===== Compute Jacobians if requested =====

    if (H_pose) {
      // Jacobian w.r.t. pose: ∂(||v_body||)/∂pose
      // v_body = R_bw * v_world, so rotation changes affect v_body
      // For small rotation perturbation δθ in world frame:
      //   δ(R_bw) ≈ -R_bw * [δθ]×
      //   δ(v_body) ≈ -R_bw * [δθ]× * v_world = -R_bw * [v_world]× * δθ
      //             = R_bw * [v_world]× * δθ  (using [a]× b = -[b]× a)
      //   ∂(v_body)/∂θ = -R_bw * [v_world]×
      //
      // For magnitude: ∂(||v||)/∂v = v^T / ||v||
      // Chain rule: ∂(||v_body||)/∂θ = (v_body^T / ||v_body||) * (-R_bw * [v_world]×)

      // Skew-symmetric matrix [v_world]×
      gtsam::Matrix3 v_world_skew = gtsam::Matrix3::Zero();
      v_world_skew(0, 1) = -vel_world.z();
      v_world_skew(0, 2) =  vel_world.y();
      v_world_skew(1, 0) =  vel_world.z();
      v_world_skew(1, 2) = -vel_world.x();
      v_world_skew(2, 0) = -vel_world.y();
      v_world_skew(2, 1) =  vel_world.x();

      // ∂(v_body)/∂θ = -R_bw * [v_world]×
      gtsam::Matrix3 dv_body_dtheta = -R_bw * v_world_skew;

      // ∂(||v_body||)/∂θ = v_body^T / ||v_body|| * dv_body_dtheta
      gtsam::Matrix H_theta = v_body.transpose() / v_body_norm * dv_body_dtheta;  // 1x3

      // GTSAM Pose3 parameterization: [rotation(3), position(3)]
      // ∂(airspeed)/∂position = 0 (airspeed doesn't depend on translation)
      gtsam::Matrix H_pose_mat = gtsam::Matrix::Zero(1, 6);
      H_pose_mat.block<1, 3>(0, 0) = H_theta;  // Rotation part
      // Position part (cols 3:6) remains zero

      *H_pose = H_pose_mat;
    }

    if (H_vel) {
      // Jacobian w.r.t. world velocity: ∂(||v_body||)/∂vel_world
      // Chain rule: ∂(||v_body||)/∂vel_world = ∂(||v_body||)/∂v_body * ∂v_body/∂vel_world
      // where ∂v_body/∂vel_world = R_bw
      //
      // ∂(||v||)/∂v = v^T / ||v||
      // So: ∂(||v_body||)/∂vel_world = (v_body^T / ||v_body||) * R_bw

      *H_vel = v_body.transpose() / v_body_norm * R_bw;  // 1x3
    }

    // Return error: predicted - measured
    return gtsam::Vector1(v_body_norm - measured_airspeed_);
  }

  /** Return a deep copy of this factor */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** Print the factor for debugging */
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "AeroVelocityFactor(meas=" << measured_airspeed_ << " m/s)\n";
    Base::print("", keyFormatter);
  }

  /** Test equality */
  bool equals(const gtsam::NonlinearFactor& other, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&other);
    return e != nullptr && Base::equals(*e, tol) &&
           std::abs(measured_airspeed_ - e->measured_airspeed_) < tol;
  }
};

}} // namespace chimera::factors
