#pragma once

#include "processors/i_sensor_processor.h"

namespace chimera {
namespace processors {

// =================== Range Altimeter Result ===================
// Extended result specifically for range-based altimeters (LiDAR, ToF)
struct RangeAltimeterResult : SensorProcessingResult {
  double range_min = 0.0;
  double range_mean = 0.0;
  double range_max = 0.0;
  int num_points = 0;

  // Quality metrics
  double variance = 0.0;
  bool stuck_detected = false;
  bool in_valid_range = false;
  bool soft_guard_triggered = false;
};

// =================== IRangeAltimeterProcessor Interface ===================
// Specialized interface for range-based altitude sensors (LiDAR, ToF)
// Provides common functionality for stuck detection, range validation, etc.
class IRangeAltimeterProcessor : public ISensorProcessor {
public:
  virtual ~IRangeAltimeterProcessor() = default;

  // Process range measurement (delegates to ISensorProcessor::process)
  SensorProcessingResult process(
    const SensorProcessingContext& ctx,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& init
  ) override {
    RangeAltimeterResult range_result = processRange(ctx, graph, init);

    // Convert to base SensorProcessingResult
    SensorProcessingResult result;
    result.factor_added = range_result.factor_added;
    result.sensor_healthy = range_result.sensor_healthy;
    result.sensor_available = range_result.sensor_available;
    result.error = range_result.error;
    result.altitude_constrained = range_result.altitude_constrained;
    result.measured_altitude = range_result.measured_altitude;
    result.sigma = range_result.sigma;

    return result;
  }

  // Range-specific processing (implemented by concrete sensors)
  virtual RangeAltimeterResult processRange(
    const SensorProcessingContext& ctx,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& init
  ) = 0;

  // Sensor characteristics
  virtual double getMinRange() const = 0;
  virtual double getMaxRange() const = 0;
  virtual double getTypicalSigma() const = 0;

  // Stuck detection
  virtual bool isStuck() const = 0;
  virtual void resetStuckGuard() = 0;

  // Confidence/quality metrics
  virtual double getConfidence() const = 0;  // 0.0 to 1.0
};

}  // namespace processors
}  // namespace chimera
