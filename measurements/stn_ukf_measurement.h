#pragma once

#include "../core/fusion/sr_ukf.h"
#include "../core/fusion/state.h"           // standard state
#include "../core/fusion/state_extended.h"  // extended state with clock
#include "stn_observation.h"
#include "stn_manager.h"
#include <memory>

namespace measurements {

class STNUKFMeasurement : public aion::MeasurementBase {
public:
  struct Params {
    double chi2_gate_threshold;   // Chi-squared threshold for gating
    double max_innovation;        // Maximum innovation allowed (units match measurement)
    bool   enable_raim;           // Enable RAIM outlier detection
    int    min_beacons;          // Minimum beacons for update
    bool   use_tdoa_geometry_for_tdoa; // Use TDOA-specific GDOP (default true)

    Params()
      : chi2_gate_threshold(25.0),
        max_innovation(100.0),
        enable_raim(true),
        min_beacons(4),
        use_tdoa_geometry_for_tdoa(true) {}
  };

  // Pass a snapshot of the current state to fix dimension at construction
  STNUKFMeasurement(const std::vector<STNObservation>& observations,
                    std::shared_ptr<STNManager> stn_manager,
                    const aion::ExtendedState& state_hint,
                    const Params& params = Params());

  // MeasurementBase interface
  Eigen::VectorXd predict(const aion::State& state) const override;
  Eigen::VectorXd getValue() const override { return z_; }
  Eigen::MatrixXd getCovariance() const override { return R_; }
  int getDimension() const override { return static_cast<int>(z_.size()); }
  double getTime() const override { return timestamp_; }

  // Extended interface for clock states
  Eigen::VectorXd predictExtended(const aion::ExtendedState& state) const;

  // Introspection
  const std::vector<STNObservation>& selectedObs() const { return selected_obs_; }
  const std::vector<std::string>& selectedIds() const { return selected_ids_; }
  GDOPResult getGDOP() const { return gdop_result_; }
  bool isValid() const { return valid_; }

  // Noise scaling for adaptive protection
  void scaleNoise(double scale_factor);

private:
  // Input data
  std::vector<STNObservation> obs_epoch_;
  std::shared_ptr<STNManager> stn_;
  Params params_;
  double timestamp_;

  // Fixed after construction
  std::vector<STNObservation> selected_obs_;
  std::vector<std::string>    selected_ids_;
  std::string                 tdoa_ref_id_;   // Reference beacon for TDOA/FDOA
  GDOPResult                  gdop_result_;
  bool                        valid_{false};
  Eigen::VectorXd             z_;             // Stacked measurement vector
  Eigen::MatrixXd             R_;             // Stacked covariance matrix

  // Helper methods
  void selectBeaconsAndAssemble(const aion::ExtendedState& xhint);
  void buildZandR();
  bool isTDOAEpoch() const;
  bool isFDOAEpoch() const;
};

} // namespace measurements