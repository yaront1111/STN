#include "stn_ukf_measurement.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <spdlog/spdlog.h>

namespace measurements {

// Helper to check if all observations have the same type
static inline bool isSameType(const std::vector<STNObservation>& v, STNMeasurementType t) {
  if (v.empty()) return false;
  for (const auto& o : v) if (o.type != t) return false;
  return true;
}

STNUKFMeasurement::STNUKFMeasurement(const std::vector<STNObservation>& observations,
                                     std::shared_ptr<STNManager> stn_manager,
                                     const aion::ExtendedState& state_hint,
                                     const Params& params)
  : obs_epoch_(observations),
    stn_(std::move(stn_manager)),
    params_(params),
    timestamp_(observations.empty() ? 0.0 : observations.front().timestamp) {

  // 1) Basic sanity: filter out invalid observations
  obs_epoch_.erase(std::remove_if(obs_epoch_.begin(), obs_epoch_.end(),
                   [](const STNObservation& o){ return !o.is_valid; }),
                   obs_epoch_.end());

  // 2) Select beacons + optional RAIM + pick ref for TDOA/FDOA
  selectBeaconsAndAssemble(state_hint);

  // 3) Build z/R and freeze dimension
  if (valid_) buildZandR();
}

bool STNUKFMeasurement::isTDOAEpoch() const {
  return isSameType(selected_obs_, STNMeasurementType::TDOA);
}

bool STNUKFMeasurement::isFDOAEpoch() const {
  return isSameType(selected_obs_, STNMeasurementType::FDOA);
}

void STNUKFMeasurement::selectBeaconsAndAssemble(const aion::ExtendedState& xhint) {
  valid_ = false;
  selected_obs_.clear();
  selected_ids_.clear();
  tdoa_ref_id_.clear();
  gdop_result_ = GDOPResult{};
  gdop_result_.gdop = 999.9;

  if (!stn_ || obs_epoch_.size() < static_cast<size_t>(params_.min_beacons)) {
    return;
  }

  // SNR filter - only keep observations with good SNR and valid beacons
  std::vector<STNObservation> valid = obs_epoch_;
  valid.erase(std::remove_if(valid.begin(), valid.end(),
               [&](const STNObservation& o){
                 return o.snr_db < stn_->params().config.snr_threshold_db ||
                        !stn_->getBeacon(o.beacon_id);
               }), valid.end());

  if (static_cast<int>(valid.size()) < params_.min_beacons) {
    return;
  }

  // Decide epoch mode (assume homogeneous type per epoch)
  const auto epoch_type = valid.front().type;
  const bool tdoa_mode = (epoch_type == STNMeasurementType::TDOA);
  const bool fdoa_mode = (epoch_type == STNMeasurementType::FDOA);

  // If TDOA/FDOA, choose reference beacon (max SNR by default)
  if (tdoa_mode || fdoa_mode) {
    auto it = std::max_element(valid.begin(), valid.end(),
            [](const STNObservation& a, const STNObservation& b){
              return a.snr_db < b.snr_db;
            });
    if (it == valid.end()) return;
    tdoa_ref_id_ = it->beacon_id;

    // Set reference beacon for all observations if not already set
    for (auto& o : valid) {
      if (o.beacon_id != tdoa_ref_id_ && o.ref_beacon_id.empty()) {
        o.ref_beacon_id = tdoa_ref_id_;
      }
    }
  }

  // Sort by SNR (descending) for geometric selection
  std::sort(valid.begin(), valid.end(),
            [](const STNObservation& a, const STNObservation& b){
              return a.snr_db > b.snr_db;
            });

  // Select beacons by geometry (TOA vs TDOA geometry)
  const int maxN = std::min<int>(static_cast<int>(valid.size()),
                                 stn_->params().config.max_beacons);
  double best_gdop = 1e9;
  std::vector<STNObservation> best_obs;

  for (int k = maxN; k >= params_.min_beacons; --k) {
    std::vector<STNObservation> candidate(valid.begin(), valid.begin() + k);
    std::vector<std::string> ids;
    std::vector<double> sigmas;
    ids.reserve(candidate.size());
    sigmas.reserve(candidate.size());

    for (const auto& o : candidate) {
      ids.push_back(o.beacon_id);
      sigmas.push_back(o.sigma);
    }

    GDOPResult gd;
    if (tdoa_mode && params_.use_tdoa_geometry_for_tdoa) {
      if (tdoa_ref_id_.empty()) continue;
      gd = stn_->computeGDOP_TDOA(xhint.position, ids, tdoa_ref_id_, &sigmas);
    } else {
      gd = stn_->computeGDOP_TOA(xhint.position, ids, &sigmas);
    }

    if (gd.gdop < best_gdop) {
      best_gdop = gd.gdop;
      best_obs = std::move(candidate);
      gdop_result_ = gd;
    }
  }

  if (static_cast<int>(best_obs.size()) < params_.min_beacons) {
    return;
  }

  // Optional RAIM for TOA/Doppler (simpler for TDOA/FDOA)
  if (params_.enable_raim && !tdoa_mode && !fdoa_mode) {
    // Simple RAIM: could implement residual-based outlier removal here
    // For now, we trust the SNR-based selection
  }

  selected_obs_ = std::move(best_obs);
  selected_ids_.clear();
  selected_ids_.reserve(selected_obs_.size());
  for (const auto& o : selected_obs_) {
    selected_ids_.push_back(o.beacon_id);
  }

  valid_ = true;
}

void STNUKFMeasurement::buildZandR() {
  if (!valid_) {
    z_.resize(0);
    R_.resize(0, 0);
    return;
  }

  const int n = static_cast<int>(selected_obs_.size());
  z_.resize(n);
  R_ = Eigen::MatrixXd::Zero(n, n);

  for (int i = 0; i < n; ++i) {
    z_(i) = selected_obs_[i].value;
    const double s = std::max(1e-3, selected_obs_[i].sigma);
    R_(i, i) = s * s;
  }

  // Adaptive noise scaling based on GDOP (continuous and aggressive)
  if (std::isfinite(gdop_result_.gdop) && gdop_result_.gdop > 1.0) {
    // Quadratic scaling for stronger sensitivity to poor geometry
    // GDOP=2 -> scale=2, GDOP=5 -> scale=12.5, GDOP=10 -> scale=50
    const double reference_gdop = 2.0;  // Ideal GDOP reference
    const double scale_factor = std::pow(gdop_result_.gdop / reference_gdop, 2.0);

    // Apply scaling with minimum factor of 1.0
    const double final_scale = std::max(1.0, scale_factor);
    R_ *= final_scale;

    // Log significant scaling events
    if (final_scale > 5.0) {
      spdlog::debug("STN: Adaptive noise scaling applied - GDOP={:.2f}, scale={:.1f}x",
                    gdop_result_.gdop, final_scale);
    }
  }
}

void STNUKFMeasurement::scaleNoise(double scale_factor) {
  // Scale measurement covariance by scale_factor^2
  // This preserves relative weights while inflating uncertainty
  const double scale_squared = scale_factor * scale_factor;
  R_ *= scale_squared;
}

Eigen::VectorXd STNUKFMeasurement::predict(const aion::State& state) const {
  // Convert standard state to extended state
  aion::ExtendedState ext;
  ext.fromStandardState(state);
  // Clock states remain at their current values (or zero if not estimated)
  return predictExtended(ext);
}

Eigen::VectorXd STNUKFMeasurement::predictExtended(const aion::ExtendedState& state) const {
  if (!valid_ || z_.size() == 0) {
    return Eigen::VectorXd::Zero(0);
  }

  const int n = static_cast<int>(z_.size());
  Eigen::VectorXd h(n);
  const double c = 299792458.0;  // Speed of light

  for (int i = 0; i < n; ++i) {
    const auto& o = selected_obs_[i];

    switch (o.type) {
      case STNMeasurementType::TOA: {
        // Time of Arrival: range + receiver clock bias (in meters)
        const auto* beacon = stn_->getBeacon(o.beacon_id);
        if (!beacon) {
          h(i) = 0.0;
          break;
        }
        const double rho = stn_->computeRange(state.position, *beacon);
        h(i) = rho + state.clock_bias_m;
        break;
      }

      case STNMeasurementType::TDOA: {
        // Time Difference of Arrival: range difference (clock cancels)
        const auto* beacon_ref = stn_->getBeacon(o.ref_beacon_id);
        const auto* beacon_i = stn_->getBeacon(o.beacon_id);
        if (!beacon_ref || !beacon_i) {
          h(i) = 0.0;
          break;
        }
        const double range_i = (beacon_i->position_ned - state.position).norm();
        const double range_ref = (beacon_ref->position_ned - state.position).norm();
        h(i) = range_i - range_ref;  // Clock bias cancels out
        break;
      }

      case STNMeasurementType::DOPPLER: {
        // Doppler shift in Hz: -(fc/c) * (u·v) + fc * clock_drift_frac
        const auto* beacon = stn_->getBeacon(o.beacon_id);
        if (!beacon) {
          h(i) = 0.0;
          break;
        }
        const Eigen::Vector3d u = stn_->computeUnitVector(state.position, *beacon);
        const double fc = beacon->frequency_mhz * 1e6;  // Convert to Hz
        const double radial_velocity = u.dot(state.velocity);
        h(i) = -(fc / c) * radial_velocity + fc * state.clock_drift_frac;
        break;
      }

      case STNMeasurementType::FDOA: {
        // Frequency Difference of Arrival: freq difference (clock drift cancels)
        const auto* beacon_i = stn_->getBeacon(o.beacon_id);
        const auto* beacon_ref = stn_->getBeacon(o.ref_beacon_id);
        if (!beacon_i || !beacon_ref) {
          h(i) = 0.0;
          break;
        }
        const Eigen::Vector3d ui = stn_->computeUnitVector(state.position, *beacon_i);
        const Eigen::Vector3d ur = stn_->computeUnitVector(state.position, *beacon_ref);
        const double fc = beacon_i->frequency_mhz * 1e6;  // Assume same frequency
        const double radial_vel_diff = (ui - ur).dot(state.velocity);
        h(i) = -(fc / c) * radial_vel_diff;  // Clock drift cancels
        break;
      }

      case STNMeasurementType::RSS:
        // Received Signal Strength not yet modeled
        h(i) = 0.0;
        break;
    }
  }

  return h;
}

} // namespace measurements