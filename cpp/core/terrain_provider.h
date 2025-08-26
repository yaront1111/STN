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
  // Tunable parameters for synthetic terrain:
  double A1 = 100.0;  // Larger amplitude for stronger signal
  double L1 = 2000.0; // Longer wavelength for broader features
  double A2 = 50.0;
  double L2 = 800.0;
  
  // SRTM reader for real terrain (optional)
  std::unique_ptr<SRTMReader> srtm_reader;
  bool use_real_terrain = false;
  
  // Reference point for NED to lat/lon conversion
  double ref_lat = 47.4;  // Zurich latitude
  double ref_lon = 8.5;   // Zurich longitude
  
  TerrainProvider() {
    // Try to initialize SRTM reader
    srtm_reader = std::make_unique<SRTMReader>("data/terrain");
    // Check if we have SRTM tiles available
    double test_elev = srtm_reader->getElevation(ref_lat, ref_lon);
    if (test_elev > -1000 && test_elev < 9000) {
      use_real_terrain = true;
      std::cout << "Using real SRTM terrain data\n";
    } else {
      std::cout << "Using synthetic terrain\n";
    }
  }

  // Convert NED position to lat/lon
  void nedToLatLon(double n, double e, double& lat, double& lon) const {
    const double lat_to_m = 111320.0;
    const double lon_to_m = 111320.0 * cos(ref_lat * M_PI / 180.0);
    
    lat = ref_lat + n / lat_to_m;
    lon = ref_lon + e / lon_to_m;
  }

  // Height function: real SRTM or synthetic
  inline TerrainSample sample(double n, double e) const {
    if (use_real_terrain && srtm_reader) {
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
    } else {
      // Synthetic terrain (original implementation)
      const double w1n = 2.0*M_PI/L1, w1e = 2.0*M_PI/L1;
      const double w2n = 2.0*M_PI/L2, w2e = 2.0*M_PI/L2;

      double h1 = A1 * std::sin(w1n*n) * std::cos(w1e*e);
      double h2 = A2 * std::cos(w2n*n + 0.4) * std::sin(w2e*e + 0.8);
      double h  = h1 + h2;

      // gradients
      double dHdn = A1 * w1n * std::cos(w1n*n) * std::cos(w1e*e)
                  - A2 * w2n * std::sin(w2n*n + 0.4) * std::sin(w2e*e + 0.8);
      double dHde = -A1 * w1e * std::sin(w1n*n) * std::sin(w1e*e)
                    + A2 * w2e * std::cos(w2n*n + 0.4) * std::cos(w2e*e + 0.8);

      return TerrainSample{h, Eigen::Vector2d(dHdn, dHde)};
    }
  }
};