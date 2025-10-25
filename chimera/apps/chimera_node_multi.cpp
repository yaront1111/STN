// CHIMERA - Multi-Sensor Real Flight Data Mode (Single-File Refactor)
//
// This is a self-contained refactor: config + loaders + helpers + FlightComputer + main().
// Requires your existing CHIMERA/GTSAM headers/libs in include paths.
//
// Build example:
//   g++ -std=c++17 chimera_node_multi_refactored.cpp -o chimera_node_multi \
//       $(pkg-config --cflags --libs gtsam) -lboost_json
//
// Run example:
//   ./chimera_node_multi --data DATA/flight_all_sensors_complete.json --flow DATA/optical_flow.json
//
// Notes:
// - Keeps original logic: LiDAR stuck detection, OF gating/health, airspeed fallback, staged priors,
//   Z-sign detection, periodic UKF re-anchor, telemetry.
// - All tunables centralized in EstimatorConfig.
// - Minor fixes/cleanups vs original (see comments).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <boost/json.hpp>

#include "core/contract_monitor.h"
#include "core/smoother_wrapper.h"
#include "core/ukf.h"

#include "factors/aero_velocity_wind_factor.h"
#include "factors/altimeter_range_factor.h"
#include "factors/baro_factor.h"
#include "factors/mag_yaw_factor.h"
#include "factors/optical_flow_factor.h"
#include "factors/velocity_magnitude_factor.h"

#include "utils/telemetry.h"
#include "utils/timing.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using gtsam::symbol_shorthand::X;  // Pose
using gtsam::symbol_shorthand::V;  // Vel
using gtsam::symbol_shorthand::B;  // Bias
using gtsam::symbol_shorthand::W;  // Wind(2D)


// =================== Core & Utils ===================
#include "core/flight_computer.h"
#include "utils/data_loaders.h"

using chimera::core::FlightComputer;
using chimera::core::EstimatorConfig;
using chimera::utils::SensorData;
using chimera::utils::loadMultiSensorData;
using chimera::utils::loadOpticalFlowData;


// =================== main ===================
int main(int argc, char** argv){
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::cout<<"CHIMERA - Multi-Sensor Real Flight Data Mode (Single-File Refactor)\n";

  std::string data_file="DATA/flight_all_sensors_complete.json";
  std::string flow_file="DATA/optical_flow.json";
  bool safe_boot=true;
  double lag_seconds = 20.0;  // Optimized: reduces marginalization losses

  for(int i=1;i<argc;++i){
    std::string a=argv[i];
    if(a=="--data" && i+1<argc){ data_file=argv[++i]; }
    else if(a=="--flow" && i+1<argc){ flow_file=argv[++i]; }
    else if(a=="--no-safe-boot"){ safe_boot=false; }
    else if(a=="--lag" && i+1<argc){ lag_seconds = std::stod(argv[++i]); }
    else if(a=="--help"){
      std::cout<<"Usage:\n"
               <<"  --data <file>   Sensor data JSON\n"
               <<"  --flow <file>   Optical flow JSON\n"
               <<"  --no-safe-boot  Disable cold-start guards\n"
               <<"  --lag <seconds> Fixed-lag window (default: 20.0, optimized for 60s missions)\n";
      return 0;
    }
  }

  try{
    SensorData s = loadMultiSensorData(data_file);
    s.optical_flow = loadOpticalFlowData(flow_file);
    if(s.imu.empty()){ std::cerr<<"No IMU data!\n"; return 1; }

    EstimatorConfig cfg;
    cfg.lag_seconds = lag_seconds;
    FlightComputer fc(cfg, std::move(s), safe_boot, "logs/chimera_multi.jsonl");
    fc.run();
  } catch(const std::exception& e){
    std::cerr<<"Error: "<<e.what()<<"\n"; return 1;
  }
  return 0;
}
