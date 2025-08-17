#pragma once
#include <Eigen/Dense>
#include <cmath>

// Synthetic terrain provider for Phase 1 (no map needed).
// Height field (meters) over N,E in meters; returns height and gradient dH/dN, dH/dE.
struct TerrainSample {
  double h;             // terrain height (m)
  Eigen::Vector2d grad; // [dH/dN, dH/dE]  (unit: m/m)
};

struct TerrainProvider {
  // Tunable parameters:
  double A1 = 100.0;  // Larger amplitude for stronger signal
  double L1 = 2000.0; // Longer wavelength for broader features
  double A2 = 50.0;
  double L2 = 800.0;

  // Height function: smooth "ridgey" landscape
  inline TerrainSample sample(double n, double e) const {
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
};