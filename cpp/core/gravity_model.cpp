#include "gravity_model.h"
#include <cmath>
#include <iostream>

// Static member definitions
std::unique_ptr<EGM2008Reader> GravityModel::egm2008_reader_ = nullptr;
bool GravityModel::initialized_ = false;

static double normal_gravity(double lat_rad, double alt_m) {
  const double gamma_e = 9.7803253359;
  const double k = 0.00193185265241;
  const double e2 = 0.00669437999014;
  double s = std::sin(lat_rad), s2 = s*s;
  double g = gamma_e * (1.0 + k*s2) / std::sqrt(1.0 - e2*s2);
  g -= 3.086e-6 * alt_m; // free-air correction
  return g;
}

void GravityModel::initialize() {
  if (!initialized_) {
    // Try to load real EGM2008 data
    egm2008_reader_ = std::make_unique<EGM2008Reader>("data/egm2008/egm2008_n360.dat");
    initialized_ = true;
    if (egm2008_reader_->isLoaded()) {
      std::cout << "Loaded REAL EGM2008 gravity model data\n";
    } else {
      std::cout << "Warning: Using synthetic gravity anomalies\n";
    }
  }
}

GravityResult GravityModel::normal(const GravityQuery& q) {
  GravityResult r{};
  r.g_mps2 = normal_gravity(q.lat_rad, q.alt_m);
  r.g_NED  = {0,0,r.g_mps2};
  r.anomaly_mgal = 0.0;
  return r;
}

GravityResult GravityModel::withAnomaly(const GravityQuery& q) {
  if (!initialized_) {
    initialize();
  }
  
  GravityResult r{};
  r.g_mps2 = normal_gravity(q.lat_rad, q.alt_m);
  
  // Get gravity anomaly from EGM2008
  if (egm2008_reader_) {
    double lat_deg = q.lat_rad * 180.0 / M_PI;
    double lon_deg = q.lon_rad * 180.0 / M_PI;
    r.anomaly_mgal = egm2008_reader_->getAnomaly(lat_deg, lon_deg);
    
    // Convert anomaly from mGal to m/s² (1 mGal = 1e-5 m/s²)
    double anomaly_mps2 = r.anomaly_mgal * 1e-5;
    r.g_mps2 += anomaly_mps2;
  }
  
  r.g_NED = {0, 0, r.g_mps2};
  return r;
}
