#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <map>
#include <memory>

namespace chimera { namespace core {

/**
 * @brief Wrapper around GTSAM's IncrementalFixedLagSmoother for GPS-denied navigation
 *
 * Manages a fixed-lag sliding window smoother that:
 * - Maintains a bounded-memory factor graph (lag_seconds window)
 * - Automatically marginalizes old variables as time progresses
 * - Provides marginal covariance extraction for re-anchoring
 * - Handles gauge freedom with optional planar XY priors
 */
class SmootherWrapper {
 public:
  using KeyTimestampMap = gtsam::FixedLagSmoother::KeyTimestampMap;

  /**
   * @brief Construct fixed-lag smoother wrapper
   * @param lag_seconds Duration of the sliding window (e.g., 7.0 seconds)
   */
  explicit SmootherWrapper(double lag_seconds);

  /**
   * @brief Add factors with automatic timestamp assignment (all keys get same timestamp)
   * @param factors New factors to add to the graph
   * @param values Initial values for new variables
   * @param t_seconds Timestamp to assign to all new keys
   *
   * NOTE: This is a convenience wrapper. All keys in 'values' will receive timestamp t_seconds.
   * If you need per-key timestamps, use update() directly.
   */
  void addFactors(const gtsam::NonlinearFactorGraph& factors,
                  const gtsam::Values& values,
                  double t_seconds);

  /**
   * @brief Update smoother with new factors and per-key timestamps
   * @param factors New factors to add
   * @param values Initial values for new variables
   * @param timestamps Map from Key to timestamp for marginalization tracking
   *
   * This is the primary update method. The IncrementalFixedLagSmoother will:
   * 1. Add new factors and values
   * 2. Optimize the active window
   * 3. Marginalize variables older than (current_max_time - lag_seconds)
   */
  void update(const gtsam::NonlinearFactorGraph& factors,
              const gtsam::Values& values,
              const KeyTimestampMap& timestamps);

  /**
   * @brief Get the current optimized estimate for all active variables
   * @return Values containing the latest optimized state
   */
  gtsam::Values calculateEstimate() const;

  /**
   * @brief Get marginal covariance for a single variable
   * @param key The key to query
   * @return Marginal covariance matrix for the specified key
   *
   * Used for re-anchoring: extract the smoother's uncertainty for a specific state
   */
  gtsam::Matrix marginalCovariance(gtsam::Key key) const;

  /**
   * @brief Get last update time
   * @return The most recent timestamp added to the smoother
   */
  double getLastTime() const { return last_time_; }

  /**
   * @brief Get the underlying smoother (for advanced queries)
   * @return Reference to the IncrementalFixedLagSmoother
   */
  const gtsam::IncrementalFixedLagSmoother& getSmoother() const { return smoother_; }

  /**
   * @brief Reinitialize smoother with new lag window (hot-reset for baro takeover)
   * @param new_lag_seconds New fixed-lag window duration
   *
   * Discards the current smoother and creates a fresh IncrementalFixedLagSmoother.
   * Used during baro takeover to purge bad altitude history from the window.
   */
  void reinit(double new_lag_seconds);

  /**
   * @brief Add planar XY gauge prior to prevent horizontal drift
   * @param key The key to apply the prior to (typically the oldest active key)
   * @param ref_pose Reference pose for the gauge
   *
   * Applies a weak prior on X-Y position only, leaving Z, roll, pitch, yaw free.
   * This prevents unbounded drift in horizontal position when absolute position sensors
   * (like GPS) are unavailable.
   */
  void addPlanarXYGaugePrior(gtsam::Key key, const gtsam::Pose3& ref_pose);

 private:
  gtsam::IncrementalFixedLagSmoother smoother_;  ///< The underlying GTSAM smoother
  double lag_seconds_;                            ///< Fixed-lag window duration (seconds)
  double last_time_ = 0.0;                        ///< Most recent timestamp added
};

}} // namespace chimera::core
