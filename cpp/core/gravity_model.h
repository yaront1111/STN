#pragma once
#include <Eigen/Dense>

struct GravityResult {
  double g_mps2;
  Eigen::Vector3d g_NED;
};
struct GravityQuery {
  double lat_rad{0.0};
  double alt_m{0.0};
};
struct GravityModel {
  static GravityResult normal(const GravityQuery& q);
};
