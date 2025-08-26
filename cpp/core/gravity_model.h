#pragma once
#include <Eigen/Dense>
#include "egm2008_reader.h"
#include <memory>

struct GravityResult {
  double g_mps2;
  Eigen::Vector3d g_NED;
  double anomaly_mgal{0.0};  // Gravity anomaly in milligals
};
struct GravityQuery {
  double lat_rad{0.0};
  double lon_rad{0.0};  // Added longitude for EGM2008
  double alt_m{0.0};
};

class GravityModel {
private:
  static std::unique_ptr<EGM2008Reader> egm2008_reader_;
  static bool initialized_;
  
public:
  static void initialize();
  static GravityResult normal(const GravityQuery& q);
  static GravityResult withAnomaly(const GravityQuery& q);
};
