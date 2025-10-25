#pragma once

#include "processors/tof_processor.h"
#include <boost/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

namespace chimera {
namespace utils {

// =================== ToF Data Loader ===================
// Loads ToF samples from JSON file
// Expected JSON format:
// [
//   {"t": 0.1, "range_mean": 1.5, "range_min": 1.48, "range_max": 1.52,
//    "num_points": 1, "confidence": 0.95, "status": 0},
//   ...
// ]
inline std::vector<processors::TofSample> loadTofData(const std::string& path){
  std::vector<processors::TofSample> samples;

  std::ifstream ifs(path);
  if(!ifs){
    std::cerr << "[ToF Loader] Warning: Could not open " << path << std::endl;
    return samples;
  }

  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

  try {
    boost::json::value jv = boost::json::parse(content);
    if(!jv.is_array()){
      std::cerr << "[ToF Loader] Error: JSON root must be array" << std::endl;
      return samples;
    }

    auto arr = jv.as_array();
    samples.reserve(arr.size());

    for(const auto& elem : arr){
      if(!elem.is_object()) continue;
      auto obj = elem.as_object();

      processors::TofSample s;
      s.t = obj.at("t").to_number<double>();
      s.range_mean = obj.at("range_mean").to_number<double>();

      // Optional fields with defaults
      s.range_min = obj.contains("range_min") ?
        obj.at("range_min").to_number<double>() : s.range_mean;
      s.range_max = obj.contains("range_max") ?
        obj.at("range_max").to_number<double>() : s.range_mean;
      s.num_points = obj.contains("num_points") ?
        static_cast<int>(obj.at("num_points").to_number<int64_t>()) : 1;
      s.confidence = obj.contains("confidence") ?
        obj.at("confidence").to_number<double>() : 1.0;
      s.status = obj.contains("status") ?
        static_cast<int>(obj.at("status").to_number<int64_t>()) : 0;

      samples.push_back(s);
    }

    std::cout << "[ToF Loader] Loaded " << samples.size() << " ToF samples from "
              << path << std::endl;

  } catch(const std::exception& e){
    std::cerr << "[ToF Loader] Error parsing JSON: " << e.what() << std::endl;
    samples.clear();
  }

  return samples;
}

// =================== ToF Sample Generation (for testing) ===================
// Generate synthetic ToF data from existing LiDAR data
// Useful for testing fusion policy when no real ToF data available
inline std::vector<processors::TofSample> generateTofFromLidar(
  const std::vector<chimera::core::LidarSample>& lidar_samples,
  double noise_scale = 1.5,     // ToF is noisier than LiDAR
  double range_limit = 4.0,     // ToF max range
  double confidence_base = 0.9  // Base confidence level
){
  std::vector<processors::TofSample> tof_samples;
  tof_samples.reserve(lidar_samples.size());

  for(const auto& lidar : lidar_samples){
    // Only generate ToF for ranges within ToF capability
    if(lidar.range_mean > range_limit) continue;

    processors::TofSample tof;
    tof.t = lidar.t;

    // Add noise to simulate ToF characteristics
    const double noise = (lidar.range_mean * 0.05) * noise_scale;  // 5% noise scaled
    tof.range_mean = lidar.range_mean;
    tof.range_min = lidar.range_mean - noise;
    tof.range_max = lidar.range_mean + noise;
    tof.num_points = 1;  // Single-zone ToF

    // Confidence decreases with range
    tof.confidence = confidence_base * (1.0 - (lidar.range_mean / range_limit) * 0.3);
    tof.confidence = std::clamp(tof.confidence, 0.5, 1.0);

    tof.status = 0;  // OK status

    tof_samples.push_back(tof);
  }

  std::cout << "[ToF Generator] Generated " << tof_samples.size()
            << " synthetic ToF samples from " << lidar_samples.size()
            << " LiDAR samples" << std::endl;

  return tof_samples;
}

}  // namespace utils
}  // namespace chimera
