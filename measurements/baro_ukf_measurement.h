#pragma once

#include "../core/fusion/sr_ukf.h"
#include "../core/fusion/state.h"
#include "measurements/baro_measurement.h"
#include <memory>

namespace measurements {

// UKF measurement adapter for barometer (MSL altitude)
class BaroUKFMeasurement : public aion::MeasurementBase {
public:
  // Pass the baro measurement + ORIGIN MSL ALTITUDE (meters) and timestamp (s)
  BaroUKFMeasurement(const BaroMeasurement& baro_meas,
                     double reference_alt_msl_m,
                     double timestamp = 0.0);
  ~BaroUKFMeasurement() override = default;

  // MeasurementBase implementation
  Eigen::VectorXd predict(const aion::State& state) const override;
  Eigen::VectorXd getValue() const override;
  Eigen::MatrixXd getCovariance() const override;
  int getDimension() const override { return 1; }
  double getTime() const override;

private:
  double altitude_msl_meas_;   // Measured altitude (MSL, m)
  double sigma_m_;             // 1-sigma (m)
  double timestamp_;           // s
  double reference_alt_msl_m_; // MSL altitude of the NED origin (QNH-ref)
};

} // namespace measurements
