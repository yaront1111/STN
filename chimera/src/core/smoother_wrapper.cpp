#include "core/smoother_wrapper.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace chimera { namespace core {

SmootherWrapper::SmootherWrapper(double lag) : lag_seconds_(lag) {
  // Configure ISAM2 parameters for the fixed-lag smoother
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;  // Less aggressive for better performance
  params.relinearizeSkip = 1;          // Relinearize every update

  // Initialize the IncrementalFixedLagSmoother
  // This replaces the manual ISAM2 + marginalization approach
  smoother_ = gtsam::IncrementalFixedLagSmoother(lag, params);
}

void SmootherWrapper::addFactors(const gtsam::NonlinearFactorGraph& factors,
                                  const gtsam::Values& values,
                                  double t_seconds) {
  // Convenience wrapper: assign the same timestamp to all new keys
  KeyTimestampMap timestamps;
  for (const auto& key_value : values) {
    timestamps[key_value.key] = t_seconds;
  }

  // Delegate to the main update method
  update(factors, values, timestamps);
}

void SmootherWrapper::update(const gtsam::NonlinearFactorGraph& factors,
                              const gtsam::Values& values,
                              const KeyTimestampMap& timestamps) {
  // The IncrementalFixedLagSmoother handles everything:
  // 1. Adds new factors and values to the graph
  // 2. Runs ISAM2 optimization on the active window
  // 3. Automatically marginalizes variables older than (max_time - lag_seconds)
  // 4. Maintains bounded memory usage
  smoother_.update(factors, values, timestamps);

  // Track the latest timestamp for monitoring
  for (const auto& kv : timestamps) {
    if (kv.second > last_time_) {
      last_time_ = kv.second;
    }
  }
}

gtsam::Values SmootherWrapper::calculateEstimate() const {
  return smoother_.calculateEstimate();
}

gtsam::Matrix SmootherWrapper::marginalCovariance(gtsam::Key key) const {
  // Extract marginal covariance for a single variable
  // This is used during re-anchoring to get the smoother's uncertainty estimate
  gtsam::Marginals marginals(smoother_.getFactors(), smoother_.calculateEstimate());
  return marginals.marginalCovariance(key);
}

void SmootherWrapper::reinit(double new_lag_seconds) {
  // Hot-reset: create fresh smoother with new lag window
  // Used during baro takeover to purge bad altitude history
  lag_seconds_ = new_lag_seconds;

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;

  smoother_ = gtsam::IncrementalFixedLagSmoother(lag_seconds_, params);
  last_time_ = 0.0;  // Reset time tracking
}

void SmootherWrapper::addPlanarXYGaugePrior(gtsam::Key key, const gtsam::Pose3& ref_pose) {
  // Apply a weak prior on X-Y position only to prevent horizontal drift
  // Pose3 covariance order: [roll, pitch, yaw, x, y, z]
  static const gtsam::Vector sigmas = (gtsam::Vector(6) <<
      1e6, 1e6, 1e6,   // Roll, Pitch, Yaw (uninformative - very large sigma)
      25.0, 25.0,      // X, Y ~ 25 m 1σ (moderate constraint over lag window)
      1e6              // Z (uninformative - baro/ToF handle altitude)
  ).finished();

  gtsam::NonlinearFactorGraph gauge_graph;
  gauge_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key, ref_pose, gtsam::noiseModel::Diagonal::Sigmas(sigmas));

  // Create empty timestamp map (prior doesn't need a timestamp)
  KeyTimestampMap empty_timestamps;

  // Create empty values (no new variables)
  gtsam::Values empty_values;

  // Add the gauge prior to the smoother
  smoother_.update(gauge_graph, empty_values, empty_timestamps);
}

}} // namespace chimera::core
