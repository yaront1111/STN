#include "gravity_model.h"
#include <cmath>

static double normal_gravity(double lat_rad, double alt_m) {
  const double gamma_e = 9.7803253359;
  const double k = 0.00193185265241;
  const double e2 = 0.00669437999014;
  double s = std::sin(lat_rad), s2 = s*s;
  double g = gamma_e * (1.0 + k*s2) / std::sqrt(1.0 - e2*s2);
  g -= 3.086e-6 * alt_m; // free-air correction
  return g;
}

GravityResult GravityModel::normal(const GravityQuery& q) {
  GravityResult r{};
  r.g_mps2 = normal_gravity(q.lat_rad, q.alt_m);
  r.g_NED  = {0,0,r.g_mps2};
  return r;
}
