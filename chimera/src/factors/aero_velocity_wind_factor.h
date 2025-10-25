#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace chimera { namespace factors {

/**
 * @brief Aerodynamic velocity (airspeed) factor WITH WIND STATE
 *
 * Models airspeed sensor measurement accounting for wind:
 *   measured_airspeed ≈ ||v_body_air|| = ||R_bw * (v_world - v_wind)||
 *
 * where v_wind = [w_x, w_y, 0] (wind in world horizontal plane)
 *
 * This is a scalar constraint (1-DOF) on velocity magnitude in the body frame.
 * Provides scale observability for velocity estimation AND wind state.
 *
 * Frame transformation:
 *   v_air = v_world - v_wind  (world-frame air velocity)
 *   v_body_air = R_bw * v_air, where R_bw = R_wb^T (from pose)
 *
 * This is a NoiseModelFactor3 connecting:
 *   - Pose3: provides world→body rotation R_wb
 *   - Vector3: WORLD-frame velocity
 *   - Point2: 2D wind state [w_x, w_y] in world frame
 */
class AeroVelocityWindFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Point2> {
 public:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Point2>;
  using This = AeroVelocityWindFactor;

 private:
  double measured_airspeed_;  ///< Measured airspeed (m/s, scalar)

 public:
  /**
   * @brief Construct airspeed factor with wind
   * @param poseKey Key for Pose3 (provides R_wb rotation)
   * @param velKey Key for world-frame velocity Vector3
   * @param windKey Key for 2D wind state Point2
   * @param measuredAirspeed Measured airspeed (m/s, scalar magnitude)
   * @param model Noise model (1-DOF, m/s)
   */
  AeroVelocityWindFactor(gtsam::Key poseKey, gtsam::Key velKey, gtsam::Key windKey,
                         double measuredAirspeed,
                         const gtsam::SharedNoiseModel& model)
      : Base(model, poseKey, velKey, windKey),
        measured_airspeed_(measuredAirspeed) {}

  virtual ~AeroVelocityWindFactor() = default;

  /**
   * @brief Evaluate error: predicted airspeed - measured airspeed
   * @param pose World pose (provides R_wb rotation)
   * @param vel_world WORLD-frame velocity (m/s)
   * @param wind 2D wind state [w_x, w_y] (m/s)
   * @param H_pose [out] Jacobian w.r.t. pose (1x6)
   * @param H_vel [out] Jacobian w.r.t. velocity (1x3)
   * @param H_wind [out] Jacobian w.r.t. wind (1x2)
   * @return 1-vector error (m/s)
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const gtsam::Vector3& vel_world,
                              const gtsam::Point2& wind,
                              boost::optional<gtsam::Matrix&> H_pose = boost::none,
                              boost::optional<gtsam::Matrix&> H_vel = boost::none,
                              boost::optional<gtsam::Matrix&> H_wind = boost::none) const override {
    // Construct 3D wind vector: [w_x, w_y, 0]
    gtsam::Vector3 v_wind_3d(wind.x(), wind.y(), 0.0);

    // Air velocity: v_air = v_world - v_wind
    gtsam::Vector3 v_air = vel_world - v_wind_3d;

    // Transform air velocity: world → body
    const gtsam::Matrix3 R_wb = pose.rotation().matrix();
    const gtsam::Matrix3 R_bw = R_wb.transpose();
    const gtsam::Vector3 v_body_air = R_bw * v_air;

    // Predicted airspeed (magnitude)
    const double v_body_air_norm = v_body_air.norm();

    // Safety: avoid division by zero in Jacobian
    if (v_body_air_norm < 1e-6) {
      if (H_pose) *H_pose = gtsam::Matrix::Zero(1, 6);
      if (H_vel) *H_vel = gtsam::Matrix::Zero(1, 3);
      if (H_wind) *H_wind = gtsam::Matrix::Zero(1, 2);
      return gtsam::Vector1(0.0 - measured_airspeed_);
    }

    // ===== Compute Jacobians if requested =====

    if (H_pose) {
      // Jacobian w.r.t. pose: ∂(||v_body_air||)/∂pose
      // v_body_air = R_bw * v_air, so rotation changes affect v_body_air
      // For small rotation perturbation δθ:
      //   ∂(v_body_air)/∂θ = -R_bw * [v_air]×

      // Skew-symmetric matrix [v_air]×
      gtsam::Matrix3 v_air_skew = gtsam::Matrix3::Zero();
      v_air_skew(0, 1) = -v_air.z();
      v_air_skew(0, 2) =  v_air.y();
      v_air_skew(1, 0) =  v_air.z();
      v_air_skew(1, 2) = -v_air.x();
      v_air_skew(2, 0) = -v_air.y();
      v_air_skew(2, 1) =  v_air.x();

      // ∂(v_body_air)/∂θ = -R_bw * [v_air]×
      gtsam::Matrix3 dv_body_air_dtheta = -R_bw * v_air_skew;

      // ∂(||v_body_air||)/∂θ = v_body_air^T / ||v_body_air|| * dv_body_air_dtheta
      gtsam::Matrix H_theta = v_body_air.transpose() / v_body_air_norm * dv_body_air_dtheta;  // 1x3

      // GTSAM Pose3 parameterization: [rotation(3), position(3)]
      gtsam::Matrix H_pose_mat = gtsam::Matrix::Zero(1, 6);
      H_pose_mat.block<1, 3>(0, 0) = H_theta;  // Rotation part
      // Position part (cols 3:6) remains zero

      *H_pose = H_pose_mat;
    }

    if (H_vel) {
      // Jacobian w.r.t. world velocity: ∂(||v_body_air||)/∂vel_world
      // Chain rule: ∂(||v_body_air||)/∂vel_world = ∂(||v_body_air||)/∂v_body_air * ∂v_body_air/∂v_air * ∂v_air/∂vel_world
      // where ∂v_air/∂vel_world = I (identity), ∂v_body_air/∂v_air = R_bw
      // ∂(||v||)/∂v = v^T / ||v||
      // So: ∂(||v_body_air||)/∂vel_world = (v_body_air^T / ||v_body_air||) * R_bw

      *H_vel = v_body_air.transpose() / v_body_air_norm * R_bw;  // 1x3
    }

    if (H_wind) {
      // Jacobian w.r.t. wind: ∂(||v_body_air||)/∂wind
      // Chain rule: ∂(||v_body_air||)/∂wind = ∂(||v_body_air||)/∂v_body_air * ∂v_body_air/∂v_air * ∂v_air/∂v_wind * ∂v_wind/∂wind
      // where ∂v_air/∂v_wind = -I, ∂v_body_air/∂v_air = R_bw
      // ∂v_wind_3d/∂wind = [1, 0; 0, 1; 0, 0]  (map 2D wind to 3D)
      // So: ∂(||v_body_air||)/∂wind = -(v_body_air^T / ||v_body_air||) * R_bw * [I_{2x2}; 0]
      //                               = -(v_body_air^T / ||v_body_air||) * R_bw(:, 0:2)

      gtsam::Matrix H_wind_mat = -(v_body_air.transpose() / v_body_air_norm * R_bw.block<3, 2>(0, 0));  // 1x2

      *H_wind = H_wind_mat;
    }

    // Return error: predicted - measured
    return gtsam::Vector1(v_body_air_norm - measured_airspeed_);
  }

  /** Return a deep copy of this factor */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** Print the factor for debugging */
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "AeroVelocityWindFactor(meas=" << measured_airspeed_ << " m/s)\n";
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
