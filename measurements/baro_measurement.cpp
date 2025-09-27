#include "measurements/baro_measurement.h"
#include <cmath>
#include <spdlog/spdlog.h>

namespace measurements {

BaroMeasurement::BaroMeasurement(const BaroConfig& config)
  : cfg_(config), sigma_(config.sigma_m) {}

bool BaroMeasurement::processMeasurement(const BaroData& d, const Eigen::Vector3d& p_ned) {
  if (!cfg_.enabled) return false;

  // 1) Convert pressure -> MSL altitude via ISA (use SEA-LEVEL temperature, not ambient)
  double alt_msl = pressureToAltitudeMSL(
      d.pressure_pa,
      cfg_.sea_level_pressure_pa,
      cfg_.sea_level_temp_K,
      cfg_.lapse_rate_Kpm,
      cfg_.gas_constant_R,
      cfg_.gravity);

  // 2) Apply simple compensation (linear model)
  alt_msl = applyTemperatureCompensation(alt_msl, d.temperature_c, d.temp_bias);
  alt_msl += d.offset_m;

  // 3) Optional low-pass
  if (cfg_.use_low_pass) {
    alt_msl = applyLowPassFilter(alt_msl, d.timestamp);
  }

  // Store final measurement (MSL)
  measurement_msl_ = alt_msl;

  // 4) Predict baro-measured MSL altitude from NED state
  const double predicted_msl = predictMSL(p_ned);

  // 5) Innovation and sigma
  innovation_ = measurement_msl_ - predicted_msl;

  // Temp-aware sigma scaling (keep simple & bounded)
  const double temp_scale = 1.0 + std::min(std::abs(d.temperature_c - 15.0) * 0.01, 0.5); // cap +50%
  sigma_ = std::max(1e-3, cfg_.sigma_m * temp_scale);

  // 6) χ² gating (1 dof)
  chi2_ = (innovation_ * innovation_) / (sigma_ * sigma_);
  is_valid_ = (chi2_ <= cfg_.chi2_gate_1d) && (std::abs(innovation_) <= cfg_.max_innovation_m);

  if (!is_valid_) {
    spdlog::debug("Baro rejected: innov={:.2f} m, chi2={:.2f}, pred={:.2f} m MSL, meas={:.2f} m MSL",
                  innovation_, chi2_, predicted_msl, measurement_msl_);
  }
  return is_valid_;
}

// Predicted baro-measured MSL altitude from state
double BaroMeasurement::predictMSL(const Eigen::Vector3d& p_ned) const {
  // NED: p_ned.z() is +Down (m). MSL altitude is origin_MSL - Down.
  return cfg_.reference_alt_msl_m - p_ned.z();
}

// ISA, lapse-rate layer (0–11 km)
double BaroMeasurement::pressureToAltitudeMSL(double p,
                                              double p0,
                                              double T0,
                                              double L,
                                              double R,
                                              double g) {
  // P = P0 * (1 - L*h/T0)^(g/(R*L))  =>  h = (T0/L) * (1 - (P/P0)^(R*L/g))
  const double ratio = std::clamp(p / p0, 1e-6, 1.0); // avoid domain issues
  const double expo  = (R * L) / g;
  return (T0 / L) * (1.0 - std::pow(ratio, expo));
}

double BaroMeasurement::altitudeMSLToPressure(double h,
                                              double p0,
                                              double T0,
                                              double L,
                                              double R,
                                              double g) {
  // P = P0 * (1 - L*h/T0)^(g/(R*L))
  const double base = std::max(1.0 - (L * h) / T0, 1e-6);
  const double expo = g / (R * L);
  return p0 * std::pow(base, expo);
}

double BaroMeasurement::calibrateSeaLevelPressure(double h_known,
                                                  double p_meas,
                                                  double T0,
                                                  double L,
                                                  double R,
                                                  double g) {
  // Invert for P0 given known h and measured P
  // p_meas = P0 * (1 - L*h/T0)^(g/(R*L))  =>  P0 = p_meas / (1 - L*h/T0)^(g/(R*L))
  const double base = std::max(1.0 - (L * h_known) / T0, 1e-6);
  const double expo = g / (R * L);
  return p_meas / std::pow(base, expo);
}

double BaroMeasurement::applyTemperatureCompensation(double altitude_m,
                                                     double temperature_c,
                                                     double temp_bias) const {
  // Simple linear model (keep it small; real compensation belongs in bias state)
  return altitude_m + (temperature_c - 15.0) * cfg_.temp_comp_factor_m_per_C + temp_bias;
}

double BaroMeasurement::applyLowPassFilter(double altitude_m, double t) {
  if (!filter_initialized_ || last_update_time_ < 0.0) {
    filtered_altitude_msl_ = altitude_m;
    last_update_time_ = t;
    filter_initialized_ = true;
    return altitude_m;
  }
  const double dt = t - last_update_time_;
  last_update_time_ = t;

  if (dt <= 0.0 || dt > 10.0) { // stale/reset
    filtered_altitude_msl_ = altitude_m;
    return altitude_m;
  }

  const double alpha = dt / (cfg_.filter_time_constant + dt);
  filtered_altitude_msl_ = alpha * altitude_m + (1.0 - alpha) * filtered_altitude_msl_;
  return filtered_altitude_msl_;
}

} // namespace measurements
