#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <Eigen/Dense>

namespace chimera {
namespace processors {

// =================== Sensor Processing Result ===================
struct SensorProcessingResult {
  bool factor_added = false;
  bool sensor_healthy = false;
  bool sensor_available = false;
  double error = 0.0;
  int rays_used = 0;
  int rays_rejected = 0;

  // For altimeter processors
  bool altitude_constrained = false;
  double measured_altitude = 0.0;
  double sigma = 0.0;
};

// =================== Sensor Processing Context ===================
struct SensorProcessingContext {
  double time_relative;    // Time relative to first IMU
  double time_absolute;    // Absolute timestamp
  int keyframe;           // Current keyframe index
  bool safe_boot;         // Cold-start mode flag
  bool boot_done;         // Boot phase complete

  gtsam::Key key_pose;
  gtsam::Key key_vel;
  gtsam::Key key_bias;
  gtsam::Key key_wind;

  gtsam::Pose3 ukf_pose;
  Eigen::Vector3d ukf_vel;
  gtsam::imuBias::ConstantBias ukf_bias;
};

// =================== ISensorProcessor Interface ===================
// Strategy pattern for modular sensor processing
class ISensorProcessor {
public:
  virtual ~ISensorProcessor() = default;

  // Process sensor data and add factors to the graph
  // Returns result struct with processing status and outputs
  virtual SensorProcessingResult process(
    const SensorProcessingContext& ctx,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& init
  ) = 0;

  // Get sensor name for logging/debugging
  virtual const char* name() const = 0;

  // Check if sensor is available at given time
  virtual bool isAvailable(double time_absolute) const = 0;
};

}  // namespace processors
}  // namespace chimera
