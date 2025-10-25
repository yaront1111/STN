#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Dense>

namespace chimera {
namespace factors {

/**
 * @brief Velocity magnitude constraint factor
 * Constrains the magnitude of velocity to be close to a target value
 * Used as a "leash" to prevent unrealistic velocity drift
 */
class VelocityMagnitudeFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  /**
   * @param key Velocity Vector3 key
   * @param target_magnitude Target velocity magnitude (m/s)
   * @param model Noise model (typically Isotropic)
   */
  VelocityMagnitudeFactor(gtsam::Key key, double target_magnitude,
                          const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, key),
        target_magnitude_(target_magnitude) {}

  /**
   * Error function: ||velocity|| - target_magnitude
   * Jacobian: d/dv (||v||) = v / ||v||
   */
  gtsam::Vector evaluateError(const gtsam::Vector3& velocity,
                               boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const double vmag = velocity.norm();

    if (H) {
      if (vmag > 1e-6) {
        // Jacobian of magnitude: d/dv (||v||) = v^T / ||v||
        *H = (gtsam::Matrix(1, 3) << velocity(0)/vmag, velocity(1)/vmag, velocity(2)/vmag).finished();
      } else {
        // Degenerate case: velocity near zero, use zero Jacobian
        *H = gtsam::Matrix::Zero(1, 3);
      }
    }

    return (gtsam::Vector1() << vmag - target_magnitude_).finished();
  }

 private:
  double target_magnitude_;  // Target velocity magnitude (m/s)
};

}  // namespace factors
}  // namespace chimera
