#pragma once
namespace chimera { namespace core {

/**
 * @brief Contract Monitor ensures critical sensor data streams remain fresh
 *
 * Monitors the recency of yaw, altitude, and velocity measurements to prevent
 * re-anchoring with stale data. The contract is only considered "OK" when:
 * 1. All three measurement types have been received at least once (initialized)
 * 2. Each measurement is within max_gap_s of the current time
 */
class ContractMonitor {
 public:
  /**
   * @brief Construct contract monitor with maximum allowable gap
   * @param max_gap_s Maximum time (seconds) between current time and last measurement
   */
  explicit ContractMonitor(double max_gap_s);

  /**
   * @brief Update the timestamp of the most recent yaw measurement
   * @param timestamp Time (seconds) of the measurement
   */
  void updateYaw(double timestamp);

  /**
   * @brief Update the timestamp of the most recent altitude measurement
   * @param timestamp Time (seconds) of the measurement
   */
  void updateAltitude(double timestamp);

  /**
   * @brief Update the timestamp of the most recent velocity measurement
   * @param timestamp Time (seconds) of the measurement
   */
  void updateVelocity(double timestamp);

  /**
   * @brief Check if the anchor cadence contract is satisfied
   * @param current_time Current wall-clock time (seconds)
   * @return true if all measurements are recent enough and initialized, false otherwise
   *
   * Returns false if:
   * - Not all measurement types have been received yet (not initialized)
   * - Any measurement is older than max_gap_s from current_time
   */
  bool isContractOk(double current_time) const;

  /**
   * @brief Check if all measurement types have been received at least once
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const { return initialized_; }

 private:
  double max_gap_s_;                                        ///< Maximum allowed gap (seconds)
  double last_yaw_t_ = 0.0;                                ///< Last yaw measurement timestamp
  double last_alt_t_ = 0.0;                                ///< Last altitude measurement timestamp
  double last_vel_t_ = 0.0;                                ///< Last velocity measurement timestamp
  bool initialized_ = false;                                ///< True when all sensors have reported
};

}} // namespace chimera::core
