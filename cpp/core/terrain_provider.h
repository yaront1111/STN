#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "srtm_reader.h"
#include <memory>

// Terrain provider that can use either synthetic or real SRTM data
// Height field (meters) over N,E in meters; returns height and gradient dH/dN, dH/dE.
struct TerrainSample {
  double h;             // terrain height (m)
  Eigen::Vector2d grad; // [dH/dN, dH/dE]  (unit: m/m)
};

struct TerrainProvider {
  // SRTM reader for real terrain ONLY
  std::unique_ptr<SRTMReader> srtm_reader;
  
  // Reference point for NED to lat/lon conversion
  double ref_lat = 47.4;  // Zurich latitude
  double ref_lon = 8.5;   // Zurich longitude
  
  TerrainProvider() {
    // Always use real SRTM terrain
    srtm_reader = std::make_unique<SRTMReader>("data/terrain");
    double test_elev = srtm_reader->getElevation(ref_lat, ref_lon);
    if (test_elev > -1000 && test_elev < 9000) {
      std::cout << "Using real SRTM terrain data\n";
    } else {
      std::cerr << "ERROR: SRTM data not available at reference point!\n";
    }
  }

  // Convert NED position to lat/lon
  void nedToLatLon(double n, double e, double& lat, double& lon) const {
    const double lat_to_m = 111320.0;
    const double lon_to_m = 111320.0 * cos(ref_lat * M_PI / 180.0);
    
    lat = ref_lat + n / lat_to_m;
    lon = ref_lon + e / lon_to_m;
  }

  // Height function: real SRTM only
  inline TerrainSample sample(double n, double e) const {
    if (!srtm_reader) {
      std::cerr << "ERROR: SRTM reader not initialized!\n";
      return TerrainSample{0.0, Eigen::Vector2d::Zero()};
    }
    
    // Convert NED to lat/lon
    double lat, lon;
    const double lat_to_m = 111320.0;
    const double lon_to_m = 111320.0 * cos(ref_lat * M_PI / 180.0);
    lat = ref_lat + n / lat_to_m;
    lon = ref_lon + e / lon_to_m;
    
    // Get elevation and gradient from SRTM
    double h = srtm_reader->getElevation(lat, lon);
    Eigen::Vector2d grad = srtm_reader->getGradient(lat, lon);
    
    return TerrainSample{h, grad};
  }
};