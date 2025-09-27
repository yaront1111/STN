#include "measurements/baro_ukf_measurement.h"
#include "core/fusion/state.h"
#include <spdlog/spdlog.h>

namespace measurements {

BaroUKFMeasurement::BaroUKFMeasurement(const BaroMeasurement& baro_meas,
                                       double reference_alt_msl_m,
                                       double timestamp)
    : altitude_msl_meas_(baro_meas.getValue()),
      sigma_m_(baro_meas.getSigma()),
      timestamp_(timestamp),
      reference_alt_msl_m_(reference_alt_msl_m) {}

Eigen::VectorXd BaroUKFMeasurement::predict(const aion::State& state) const {
  // Predicted baro reading is MSL altitude:
  // NED: z is +Down. MSL = origin_MSL - z_NED  (+ optional baro bias in state)
  Eigen::VectorXd z_pred(1);
  double baro_bias_m = 0.0;

  // If/when you add `double baro_bias_m` to State, uncomment:
  // baro_bias_m = state.baro_bias_m;

  z_pred(0) = reference_alt_msl_m_ - state.position.z() + baro_bias_m;
  return z_pred;
}

Eigen::VectorXd BaroUKFMeasurement::getValue() const {
  Eigen::VectorXd z(1);
  z(0) = altitude_msl_meas_;
  return z;
}

Eigen::MatrixXd BaroUKFMeasurement::getCovariance() const {
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = sigma_m_ * sigma_m_;
  return R;
}

double BaroUKFMeasurement::getTime() const {
  return timestamp_;
}

} // namespace measurements
