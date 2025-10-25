#include "utils/data_loaders.h"
#include <boost/json.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace chimera {
namespace utils {

SensorData loadMultiSensorData(const std::string& file){
  std::ifstream f(file);
  if(!f.is_open()){
    throw std::runtime_error("Cannot open " + file);
  }

  std::stringstream ss;
  ss << f.rdbuf();
  auto root = boost::json::parse(ss.str()).as_object();

  SensorData d;

  // Load IMU
  if(root.contains("imu")){
    auto a = root["imu"].as_array();
    d.imu.reserve(a.size());
    for(const auto& it: a){
      auto o = it.as_object();
      ImuSample s;
      s.t = o["t"].as_double();

      auto g = o["gyro"].as_array();
      s.gyro = {g[0].as_double(), g[1].as_double(), g[2].as_double()};

      auto ac = o["accel"].as_array();
      s.accel = {ac[0].as_double(), ac[1].as_double(), ac[2].as_double()};

      auto q = o["quat"].as_array();
      s.quat = {q[0].as_double(), q[1].as_double(), q[2].as_double(), q[3].as_double()};

      d.imu.push_back(s);
    }
  }

  // Load Mag
  if(root.contains("mag")){
    auto a = root["mag"].as_array();
    d.mag.reserve(a.size());
    for(const auto& it: a){
      auto o = it.as_object();
      MagSample s;
      s.t = o["t"].as_double();

      auto m = o["mag"].as_array();
      s.mag = {m[0].as_double(), m[1].as_double(), m[2].as_double()};

      d.mag.push_back(s);
    }
  }

  // Load LiDAR
  if(root.contains("lidar")){
    auto a = root["lidar"].as_array();
    d.lidar.reserve(a.size());
    for(const auto& it: a){
      auto o = it.as_object();
      LidarSample s;
      s.t = o["t"].as_double();
      s.range_min = o["range_min"].as_double();
      s.range_mean = o["range_mean"].as_double();
      s.num_points = static_cast<int>(o["num_points"].as_int64());

      d.lidar.push_back(s);
    }
  }

  // Load Baro
  if(root.contains("baro")){
    auto a = root["baro"].as_array();
    d.baro.reserve(a.size());
    for(const auto& it: a){
      auto o = it.as_object();
      BaroSample s;
      s.t = o["t"].as_double();
      s.altitude = o["altitude"].as_double();

      d.baro.push_back(s);
    }
  }

  // Load Airspeed
  if(root.contains("airspeed")){
    auto a = root["airspeed"].as_array();
    d.airspeed.reserve(a.size());
    for(const auto& it: a){
      auto o = it.as_object();
      AirspeedSample s;
      s.t = o["t"].as_double();
      s.airspeed = o["airspeed"].as_double();

      d.airspeed.push_back(s);
    }
  }

  std::cout << "Loaded:\n"
            << "  IMU " << d.imu.size() << "\n"
            << "  Mag " << d.mag.size() << "\n"
            << "  LiDAR " << d.lidar.size() << "\n"
            << "  Baro " << d.baro.size() << "\n"
            << "  Airspeed " << d.airspeed.size() << "\n";

  if(!d.imu.empty()){
    std::cout << "  Duration " << (d.imu.back().t - d.imu.front().t) << " s\n";
  }

  return d;
}

std::vector<OpticalFlowSample> loadOpticalFlowData(const std::string& file){
  std::ifstream f(file);
  if(!f.is_open()){
    std::cerr << "Warn: cannot open " << file << "\n";
    return {};
  }

  std::stringstream ss;
  ss << f.rdbuf();
  auto arr = boost::json::parse(ss.str()).as_array();

  std::vector<OpticalFlowSample> out;
  out.reserve(arr.size());

  for(const auto& it: arr){
    auto o = it.as_object();
    OpticalFlowSample s;
    s.t = o["t"].as_double();
    s.dt = o["dt"].as_double();
    s.rays_used = static_cast<int>(o["rays_used"].as_int64());
    s.rays_rejected = static_cast<int>(o["rays_rejected"].as_int64());

    auto rays = o["rays"].as_array();
    for(const auto& rit: rays){
      auto ro = rit.as_object();
      OpticalFlowRay r;
      r.grid_idx = static_cast<int>(ro["grid_idx"].as_int64());
      r.norm_x = ro["norm_x"].as_double();
      r.norm_y = ro["norm_y"].as_double();
      r.px = static_cast<int>(ro["px"].as_int64());
      r.py = static_cast<int>(ro["py"].as_int64());

      const double fu = ro["flow_u"].as_double();
      const double fv = ro["flow_v"].as_double();
      r.flow_u = (s.dt > 1e-6) ? (fu / s.dt) : 0.0;  // px/s
      r.flow_v = (s.dt > 1e-6) ? (fv / s.dt) : 0.0;  // px/s

      r.texture_var = ro["texture_var"].as_double();

      s.rays.push_back(r);
    }

    out.push_back(std::move(s));
  }

  std::cout << "OpticalFlow: " << out.size() << " measurements\n";
  return out;
}

}  // namespace utils
}  // namespace chimera
