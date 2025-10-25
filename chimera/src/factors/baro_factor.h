#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace chimera {
namespace factors {

/**
 * @brief Barometer altitude factor
 * Constrains the z-component of pose to match pressure-derived altitude
 */
class BaroFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  /**
   * @param key Pose3 key
   * @param z_measured Measured altitude from barometer (meters)
   * @param model Noise model (typically Gaussian or Huber-wrapped)
   */
  BaroFactor(gtsam::Key key, double z_measured, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), z_measured_(z_measured) {}

  /**
   * Error function: pose.z() - z_measured
   * Jacobian: [0,0,0, 0,0,1] (only z translation matters)
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                               boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      *H = (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 1).finished();
    }
    return (gtsam::Vector1() << pose.z() - z_measured_).finished();
  }

 private:
  double z_measured_;  // Barometer altitude measurement
};

}  // namespace factors
}  // namespace chimera
