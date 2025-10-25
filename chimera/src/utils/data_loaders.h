#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace chimera {
namespace utils {

// =================== Raw Sensor Structs ===================

struct ImuSample {
  double t;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
  Eigen::Vector4d quat;
};

struct MagSample {
  double t;
  Eigen::Vector3d mag;
};

struct LidarSample {
  double t;
  double range_min;
  double range_mean;
  int num_points;
};

struct BaroSample {
  double t;
  double altitude;
};

struct AirspeedSample {
  double t;
  double airspeed;
};

struct OpticalFlowRay {
  int grid_idx;
  double norm_x;
  double norm_y;
  int px;
  int py;
  double flow_u;  // px/s
  double flow_v;  // px/s
  double texture_var;
};

struct OpticalFlowSample {
  double t;
  double dt;
  std::vector<OpticalFlowRay> rays;
  int rays_used = 0;
  int rays_rejected = 0;
};

struct SensorData {
  std::vector<ImuSample> imu;
  std::vector<MagSample> mag;
  std::vector<LidarSample> lidar;
  std::vector<BaroSample> baro;
  std::vector<AirspeedSample> airspeed;
  std::vector<OpticalFlowSample> optical_flow;
};

// =================== JSON Loaders ===================

// Load multi-sensor data from JSON file
SensorData loadMultiSensorData(const std::string& file);

// Load optical flow data from JSON file
std::vector<OpticalFlowSample> loadOpticalFlowData(const std::string& file);

}  // namespace utils
}  // namespace chimera
