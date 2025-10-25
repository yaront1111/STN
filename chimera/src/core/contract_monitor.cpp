#include "core/contract_monitor.h"
#include <algorithm>

namespace chimera { namespace core {

ContractMonitor::ContractMonitor(double max_gap) : max_gap_s_(max_gap) {}

void ContractMonitor::updateYaw(double t) {
  last_yaw_t_ = t;
  // Check if we're now fully initialized
  if (!initialized_ && last_yaw_t_ > 0.0 && last_alt_t_ > 0.0 && last_vel_t_ > 0.0) {
    initialized_ = true;
  }
}

void ContractMonitor::updateAltitude(double t) {
  last_alt_t_ = t;
  // Check if we're now fully initialized
  if (!initialized_ && last_yaw_t_ > 0.0 && last_alt_t_ > 0.0 && last_vel_t_ > 0.0) {
    initialized_ = true;
  }
}

void ContractMonitor::updateVelocity(double t) {
  last_vel_t_ = t;
  // Check if we're now fully initialized
  if (!initialized_ && last_yaw_t_ > 0.0 && last_alt_t_ > 0.0 && last_vel_t_ > 0.0) {
    initialized_ = true;
  }
}

bool ContractMonitor::isContractOk(double current_time) const {
  // Cannot satisfy contract until all sensors have provided at least one measurement
  if (!initialized_) {
    return false;
  }

  // Check that each measurement stream is within the maximum allowed gap from current time
  bool yaw_ok = (current_time - last_yaw_t_) <= max_gap_s_;
  bool alt_ok = (current_time - last_alt_t_) <= max_gap_s_;
  bool vel_ok = (current_time - last_vel_t_) <= max_gap_s_;

  return yaw_ok && alt_ok && vel_ok;
}

}} // namespace chimera::core
