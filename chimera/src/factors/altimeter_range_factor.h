#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace chimera {
namespace factors {

/**
 * @brief Altimeter (ToF/radar) range factor for Above Ground Level (AGL)
 * Constrains the z-component of pose to match measured AGL distance
 */
class AltimeterRangeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  /**
   * @param key Pose3 key
   * @param z_agl_measured Measured AGL distance from ToF/radar (meters)
   * @param model Noise model (typically Gaussian or Huber-wrapped)
   */
  AltimeterRangeFactor(gtsam::Key key, double z_agl_measured,
                       const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), z_agl_measured_(z_agl_measured) {}

  /**
   * Error function: pose.z() - z_agl_measured
   * (MVP: assumes flat ground; later can subtract DEM(x,y) for terrain-relative AGL)
   * Jacobian: [0,0,0, 0,0,1] (only z translation matters)
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                               boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // MVP: AGL ≈ pose.z() (assumes ground at z=0)
    // Future: z_ground = DEM.lookup(pose.x(), pose.y()); error = pose.z() - z_ground - z_agl_measured
    if (H) {
      *H = (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 1).finished();
    }
    return (gtsam::Vector1() << pose.z() - z_agl_measured_).finished();
  }

 private:
  double z_agl_measured_;  // ToF/radar AGL measurement (meters)
};

}  // namespace factors
}  // namespace chimera
