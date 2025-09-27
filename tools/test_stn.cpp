#include <iostream>
#include <memory>
#include "../measurements/stn_manager.h"
#include "../measurements/stn_synthetic_generator.h"
#include "../measurements/stn_ukf_measurement.h"
#include "../core/fusion/state_extended.h"

using namespace measurements;

int main() {
  std::cout << "STN (Signals of Opportunity) Test Program\n";
  std::cout << "==========================================\n\n";

  // Initialize STN Manager
  STNManager::Params mgr_params;
  mgr_params.beacons_file = "configs/stn/beacons.yaml";
  mgr_params.origin_lat_deg = 31.7683;
  mgr_params.origin_lon_deg = 35.2137;
  mgr_params.origin_alt_msl = 750.0;

  auto stn_manager = std::make_shared<STNManager>(mgr_params);

  // Load beacons
  if (!stn_manager->loadBeacons()) {
    std::cerr << "Failed to load beacons from " << mgr_params.beacons_file << std::endl;
    return 1;
  }

  auto beacons = stn_manager->getActiveBeacons();
  std::cout << "Loaded " << beacons.size() << " active beacons:\n";
  for (const auto& beacon : beacons) {
    std::cout << "  - " << beacon.id << " (" << beacon.name << "): "
              << beacon.type << " at " << beacon.frequency_mhz << " MHz\n";
  }

  // Initialize synthetic data generator
  STNSyntheticGenerator::Params gen_params;
  gen_params.true_clock_bias_m = 100.0;
  gen_params.true_clock_drift_frac = 1e-9; // 1 ppb clock drift
  gen_params.tdoa_noise_m = 15.0;
  gen_params.min_elevation_deg = -10.0; // Allow beacons below horizon for testing

  STNSyntheticGenerator generator(stn_manager, gen_params);

  // Simulate a true state
  aion::ExtendedState true_state;
  true_state.position = Eigen::Vector3d(100, 50, 50);  // 100m N, 50m E, 50m down (above ground)
  true_state.velocity = Eigen::Vector3d(10, 5, 0);       // Moving NE at ~11 m/s
  true_state.clock_bias_m = 0.0;  // Will be estimated
  true_state.clock_drift_frac = 0.0;

  std::cout << "\n\nTest 1: Generate synthetic observations\n";
  std::cout << "----------------------------------------\n";
  std::cout << "True position (NED): " << true_state.position.transpose() << " m\n";
  std::cout << "True velocity (NED): " << true_state.velocity.transpose() << " m/s\n";
  std::cout << "True clock bias: " << generator.getTrueClockBias() << " m\n";

  // Generate synthetic observations
  double current_time = 10.0;
  auto observations = generator.generateObservations(true_state, current_time);

  std::cout << "\nGenerated " << observations.size() << " observations:\n";
  for (const auto& obs : observations) {
    std::string type_str = (obs.type == STNMeasurementType::TDOA) ? "TDOA" :
                          (obs.type == STNMeasurementType::DOPPLER) ? "Doppler" : "RSS";
    std::cout << "  - " << obs.beacon_id << " (" << type_str << "): "
              << obs.value << " ± " << obs.sigma
              << " (SNR: " << obs.snr_db << " dB)\n";
  }

  std::cout << "\n\nTest 2: GDOP computation\n";
  std::cout << "------------------------\n";

  std::vector<std::string> beacon_subset;
  for (const auto& obs : observations) {
    if (std::find(beacon_subset.begin(), beacon_subset.end(), obs.beacon_id) == beacon_subset.end()) {
      beacon_subset.push_back(obs.beacon_id);
    }
  }

  auto gdop_result = stn_manager->computeGDOP(true_state.position, beacon_subset);
  std::cout << "GDOP: " << gdop_result.gdop << "\n";
  std::cout << "PDOP: " << gdop_result.pdop << " (3D position)\n";
  std::cout << "HDOP: " << gdop_result.hdop << " (horizontal)\n";
  std::cout << "VDOP: " << gdop_result.vdop << " (vertical)\n";
  std::cout << "TDOP: " << gdop_result.tdop << " (time)\n";
  std::cout << "Geometry " << (gdop_result.is_acceptable() ? "ACCEPTABLE" : "POOR") << "\n";

  std::cout << "\n\nTest 3: Create UKF measurement\n";
  std::cout << "------------------------------\n";

  STNUKFMeasurement::Params meas_params;
  meas_params.chi2_gate_threshold = 25.0;
  meas_params.enable_raim = true;
  meas_params.min_beacons = 4;

  // Test with estimated state (with some error)
  aion::ExtendedState estimated_state = true_state;
  estimated_state.position += Eigen::Vector3d(20, -10, 5);  // Add position error
  estimated_state.clock_bias_m = 80.0;  // Clock bias estimate with error

  // Create measurement with state hint for stable dimension
  STNUKFMeasurement stn_measurement(observations, stn_manager, estimated_state, meas_params);

  auto predicted = stn_measurement.predictExtended(estimated_state);
  auto measured = stn_measurement.getValue();

  if (stn_measurement.isValid()) {
    std::cout << "Measurement is VALID\n";
    std::cout << "Selected " << stn_measurement.selectedIds().size() << " beacons\n";
    std::cout << "Measurement dimension: " << stn_measurement.getDimension() << "\n";

    if (predicted.size() == measured.size() && predicted.size() > 0) {
      Eigen::VectorXd innovation = measured - predicted;
      std::cout << "Innovation norm: " << innovation.norm() << " m\n";

      std::cout << "\nInnovations by beacon:\n";
      const auto& selected = stn_measurement.selectedIds();
      for (int i = 0; i < innovation.size() && i < static_cast<int>(selected.size()); ++i) {
        std::cout << "  - " << selected[i] << ": " << innovation(i) << " m\n";
      }
    }
  } else {
    std::cout << "Measurement is INVALID (insufficient beacons or poor geometry)\n";
  }

  std::cout << "\n\nTest 4: RAIM outlier detection\n";
  std::cout << "------------------------------\n";

  // Add an outlier observation
  if (!observations.empty()) {
    observations[0].value += 200.0;  // Add 200m error to first observation
    std::cout << "Added 200m outlier to " << observations[0].beacon_id << "\n";
  }

  Eigen::Vector2d clock_estimate(estimated_state.clock_bias_m, estimated_state.clock_drift_frac);
  auto good_beacons = stn_manager->performRAIM(estimated_state.position, observations, clock_estimate);

  std::cout << "RAIM kept " << good_beacons.size() << " of " << beacon_subset.size() << " beacons\n";
  if (good_beacons.size() < beacon_subset.size()) {
    std::cout << "Outliers detected and removed!\n";
  }

  std::cout << "\n✅ STN system test completed successfully!\n";

  return 0;
}