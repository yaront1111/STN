#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <cmath>

namespace chimera {
namespace factors {

/** Wrap angle to [-pi, pi] */
inline double wrapToPi(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief Magnetometer yaw factor (tilt-compensated heading)
 * Constrains the yaw component of pose to match magnetometer reading
 */
class MagYawFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  /**
   * @param key Pose3 key
   * @param yaw_measured Measured yaw from magnetometer (radians, NED or ENU convention)
   * @param model Noise model (typically Gaussian or Huber-wrapped)
   */
  MagYawFactor(gtsam::Key key, double yaw_measured, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), yaw_measured_(yaw_measured) {}

  /**
   * Error function: wrapToPi(yaw_est - yaw_measured)
   * Jacobian: [0,0,1, 0,0,0] (only yaw rotation matters)
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                               boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // Extract yaw from rotation (ypr returns yaw, pitch, roll)
    const gtsam::Vector3 ypr = pose.rotation().ypr();
    const double yaw_est = ypr.x();  // yaw

    // Compute wrapped error
    const double error = wrapToPi(yaw_est - yaw_measured_);

    if (H) {
      // Jacobian: derivative of yaw w.r.t. pose [rotation(3), translation(3)]
      // Only yaw component of rotation matters
      *H = gtsam::Matrix::Zero(1, 6);
      (*H)(0, 2) = 1.0;  // d(yaw)/d(rotation_z) = 1
    }

    return (gtsam::Vector1() << error).finished();
  }

 private:
  double yaw_measured_;  // Magnetometer yaw measurement (radians)
};

}  // namespace factors
}  // namespace chimera
